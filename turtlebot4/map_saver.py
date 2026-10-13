"""Robust map saver wrapper using Nav2 map_saver_cli with atomic validation."""

import argparse
from collections import Counter
import hashlib
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import yaml

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


# Nav2 trinary PGM encodes free/occupied/unknown as 254/0/205.
# Unknown has occupancy 50/255, so the upstream saver default 0.25 is unsafe.
DEFAULT_FREE_THRESH = 0.196
DEFAULT_OCC_THRESH = 0.65


def validate_thresholds(free_thresh: float, occ_thresh: float) -> None:
    if not (math.isfinite(free_thresh) and math.isfinite(occ_thresh)
            and 0.0 < free_thresh < occ_thresh < 1.0):
        raise ValueError('Occupancy thresholds must satisfy 0 < free_thresh < occupied_thresh < 1')


def classify_pgm_pixel(pixel: int, free_thresh: float, occ_thresh: float,
                       negate: bool = False) -> int:
    """Match Nav2 trinary loader's strict comparisons for 8-bit grayscale."""
    occupancy = pixel / 255.0 if negate else 1.0 - pixel / 255.0
    if occupancy > occ_thresh:
        return 100
    if occupancy < free_thresh:
        return 0
    return -1


def occupancy_signature(values) -> str:
    """Hash cell order as well as classes; -1/0/100 encode as 0/1/101."""
    return hashlib.sha256(bytes(value + 1 for value in values)).hexdigest()


def bounded_timeout(value: str) -> float:
    duration = float(value)
    if not math.isfinite(duration) or not 1.0 <= duration <= 120.0:
        raise argparse.ArgumentTypeError('timeout must be finite and between 1 and 120 seconds')
    return duration


def map_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def validate_occupancy_grid_msg(msg: OccupancyGrid) -> dict:
    """Validate live OccupancyGrid metadata and contents, rejecting empty maps."""
    info = msg.info
    if not math.isfinite(info.resolution) or info.resolution <= 0.0:
        raise ValueError(f'Invalid map resolution: {info.resolution}')
    if info.width <= 0 or info.height <= 0:
        raise ValueError(f'Invalid map dimensions: {info.width}x{info.height}')

    expected_len = info.width * info.height
    actual_len = len(msg.data)
    if actual_len != expected_len:
        raise ValueError(f'Map data length {actual_len} does not match {info.width}x{info.height}={expected_len}')

    # Count cell values
    unknown_count = 0
    free_count = 0
    occupied_count = 0
    invalid_count = 0

    for val in msg.data:
        if val == -1:
            unknown_count += 1
        elif 0 <= val <= 25:
            free_count += 1
        elif 25 < val <= 100:
            occupied_count += 1
        else:
            invalid_count += 1

    if invalid_count > 0:
        raise ValueError(f'Occupancy grid contains {invalid_count} out-of-range cell values (expected [-1, 100])')

    if unknown_count == expected_len:
        raise ValueError('Map validation rejected: 100% of cells are unknown (-1). Map is empty.')

    if free_count == 0:
        raise ValueError('Map validation rejected: 0 free cells observed. Exploration is required before saving.')

    return {
        'resolution': info.resolution,
        'width': info.width,
        'height': info.height,
        'origin': [info.origin.position.x, info.origin.position.y, info.origin.position.z],
        'total_cells': expected_len,
        'free_cells': free_count,
        'occupied_cells': occupied_count,
        'unknown_cells': unknown_count,
        'free_ratio': free_count / expected_len,
        'occupied_ratio': occupied_count / expected_len,
        'unknown_ratio': unknown_count / expected_len,
    }


def parse_pgm_header(filepath: Path | str) -> tuple[int, int, int, int]:
    """Parse a binary P5 PGM file header, returning (width, height, maxval, header_offset)."""
    with open(filepath, 'rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError(f'Invalid PGM magic number: {magic} (expected P5)')

        tokens = []
        while len(tokens) < 3:
            line = f.readline()
            if not line:
                break
            # Skip comments
            comment_idx = line.find(b'#')
            if comment_idx != -1:
                line = line[:comment_idx]
            for part in line.split():
                if part:
                    tokens.append(int(part))

        if len(tokens) != 3:
            raise ValueError(f'Failed to parse PGM header tokens from {filepath}')

        width, height, maxval = tokens
        header_offset = f.tell()
        return width, height, maxval, header_offset


def validate_saved_map_files(yaml_path: Path, pgm_path: Path) -> dict:
    """Verify saved YAML and PGM file contents and mutual consistency."""
    if not yaml_path.is_file() or yaml_path.stat().st_size == 0:
        raise FileNotFoundError(f'Map YAML file missing or empty: {yaml_path}')
    if not pgm_path.is_file() or pgm_path.stat().st_size == 0:
        raise FileNotFoundError(f'Map PGM image missing or empty: {pgm_path}')

    with open(yaml_path, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f)

    if not isinstance(meta, dict):
        raise ValueError(f'Invalid YAML content in {yaml_path}')

    image_ref = meta.get('image')
    if not image_ref:
        raise ValueError(f'YAML missing "image" key: {yaml_path}')

    # Image reference should resolve relative to YAML directory
    resolved_image = (yaml_path.parent / image_ref).resolve()
    if not resolved_image.is_file():
        raise FileNotFoundError(f'Image referenced in YAML does not exist: {resolved_image}')

    resolution = float(meta.get('resolution', 0.0))
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError(f'Invalid resolution in YAML: {resolution}')

    origin = meta.get('origin')
    if (not isinstance(origin, list) or len(origin) != 3
            or not all(math.isfinite(float(value)) for value in origin)):
        raise ValueError(f'Invalid origin in YAML: {origin}')

    if resolved_image != pgm_path.resolve():
        raise ValueError('YAML image reference does not match the validated PGM')
    if meta.get('mode', 'trinary') != 'trinary':
        raise ValueError('Assignment map requires trinary mode')
    negate = meta.get('negate')
    if negate not in (0, 1):
        raise ValueError('Map negate must be 0 or 1')
    free_thresh = float(meta.get('free_thresh', float('nan')))
    occ_thresh = float(meta.get('occupied_thresh', float('nan')))
    validate_thresholds(free_thresh, occ_thresh)

    width, height, maxval, header_offset = parse_pgm_header(pgm_path)
    if width <= 0 or height <= 0 or maxval != 255:
        raise ValueError('Map requires positive PGM dimensions and 8-bit maxval 255')
    file_size = pgm_path.stat().st_size
    expected_data_size = width * height
    actual_data_size = file_size - header_offset

    if actual_data_size != expected_data_size:
        raise ValueError(
            f'PGM data size {actual_data_size} does not match {width}x{height}={expected_data_size}'
        )

    pixels = pgm_path.read_bytes()[header_offset:]
    if 205 in pixels and classify_pgm_pixel(205, free_thresh, occ_thresh, negate) != -1:
        raise ValueError('PGM unknown pixel 205 must remain Unknown on Nav2 reload')
    classes = [classify_pgm_pixel(pixel, free_thresh, occ_thresh, negate) for pixel in pixels]
    counts = Counter(classes)
    # PGM rows run top-to-bottom; OccupancyGrid rows run bottom-to-top.
    grid = [value for row in range(height - 1, -1, -1)
            for value in classes[row * width:(row + 1) * width]]

    return {
        'free_cells': counts[0],
        'occupied_cells': counts[100],
        'unknown_cells': counts[-1],
        'occupancy_sha256': occupancy_signature(grid),
        'free_thresh': free_thresh,
        'occupied_thresh': occ_thresh,
        'resolution': resolution,
        'origin': origin,
        'image': image_ref,
        'width': width,
        'height': height,
        'maxval': maxval,
        'yaml_path': str(yaml_path),
        'pgm_path': str(pgm_path),
    }


class MapListener(Node):
    """Subscribe to map topic and capture the first valid OccupancyGrid."""

    def __init__(self, topic: str):
        super().__init__('map_save_listener')
        self.latest_map = None
        self.sub = self.create_subscription(
            OccupancyGrid, topic, self._map_cb, map_qos()
        )

    def _map_cb(self, msg: OccupancyGrid):
        self.latest_map = msg


def capture_live_map(topic: str = '/map', timeout: float = 15.0) -> OccupancyGrid:
    """Capture live OccupancyGrid message with bounded timeout."""
    listener = MapListener(topic)
    start_time = time.monotonic()
    try:
        while time.monotonic() - start_time < timeout:
            rclpy.spin_once(listener, timeout_sec=0.2)
            if listener.latest_map is not None:
                return listener.latest_map
    finally:
        listener.destroy_node()

    raise TimeoutError(
        f"Timed out ({timeout}s) waiting for live OccupancyGrid on topic '{topic}'. "
        "Ensure SLAM Toolbox is running (ros2 run turtlebot4 start_slam)."
    )


def transactional_map_replace(
    temp_yaml: Path,
    temp_pgm: Path,
    target_yaml: Path,
    target_pgm: Path,
    simulate_failure: bool = False,
) -> dict:
    """Transactionally replace destination map files with backup and automatic rollback on failure."""
    target_dir = target_yaml.parent
    target_dir.mkdir(parents=True, exist_ok=True)

    # 1. Pre-replacement validation on source temp files
    validate_saved_map_files(temp_yaml, temp_pgm)

    # 2. Stage new files in destination directory using hidden .tmp files
    stage_yaml = target_dir / f'.{target_yaml.name}.tmp'
    stage_pgm = target_dir / f'.{target_pgm.name}.tmp'

    # Backup existing destination files if they exist
    backup_yaml = target_dir / f'.{target_yaml.name}.bak'
    backup_pgm = target_dir / f'.{target_pgm.name}.bak'

    has_orig_yaml = target_yaml.is_file()
    has_orig_pgm = target_pgm.is_file()

    try:
        if has_orig_yaml:
            shutil.copy2(target_yaml, backup_yaml)
        if has_orig_pgm:
            shutil.copy2(target_pgm, backup_pgm)

        shutil.copy2(temp_yaml, stage_yaml)
        shutil.copy2(temp_pgm, stage_pgm)

        if simulate_failure:
            raise IOError("Simulated I/O failure during transactional map replacement")

        # Atomic replacement on same filesystem
        os.replace(stage_yaml, target_yaml)
        os.replace(stage_pgm, target_pgm)

        # Post-replacement validation on final destination files
        verified_info = validate_saved_map_files(target_yaml, target_pgm)

        # Clean up backups on success
        if backup_yaml.is_file():
            backup_yaml.unlink()
        if backup_pgm.is_file():
            backup_pgm.unlink()

        return verified_info

    except Exception as exc:
        # Rollback: restore from backup or remove corrupted/partial destination
        if stage_yaml.is_file():
            stage_yaml.unlink()
        if stage_pgm.is_file():
            stage_pgm.unlink()

        if has_orig_yaml and backup_yaml.is_file():
            os.replace(backup_yaml, target_yaml)
        elif not has_orig_yaml and target_yaml.is_file():
            target_yaml.unlink()

        if has_orig_pgm and backup_pgm.is_file():
            os.replace(backup_pgm, target_pgm)
        elif not has_orig_pgm and target_pgm.is_file():
            target_pgm.unlink()

        raise RuntimeError(
            f"Transactional map replacement failed and was rolled back cleanly: {exc}"
        ) from exc


def save_map_atomic(
    output_dir: Path | str | None = None,
    map_name: str = 'map',
    map_topic: str = '/map',
    timeout: float = 20.0,
    free_thresh: float = DEFAULT_FREE_THRESH,
    occ_thresh: float = DEFAULT_OCC_THRESH,
) -> dict:
    """Safely and atomically save live map using Nav2 map_saver_cli."""
    validate_thresholds(free_thresh, occ_thresh)
    if [classify_pgm_pixel(p, free_thresh, occ_thresh) for p in (254, 0, 205)] != [0, 100, -1]:
        raise ValueError('Saver thresholds must preserve free/occupied/unknown PGM pixels')
    out_dir = Path(output_dir) if output_dir else Path(__file__).resolve().parents[1] / 'maps'
    out_dir.mkdir(parents=True, exist_ok=True)
    target_yaml = out_dir / f'{map_name}.yaml'
    target_pgm = out_dir / f'{map_name}.pgm'

    print(f"[save_map] Listening for live OccupancyGrid on '{map_topic}'...")
    live_msg = capture_live_map(topic=map_topic, timeout=timeout)
    stats = validate_occupancy_grid_msg(live_msg)
    print(
        f"[save_map] Live map validated: {stats['width']}x{stats['height']} cells "
        f"({stats['width']*stats['resolution']:.1f}m x {stats['height']*stats['resolution']:.1f}m), "
        f"Free: {stats['free_cells']} ({stats['free_ratio']*100:.1f}%), "
        f"Occupied: {stats['occupied_cells']} ({stats['occupied_ratio']*100:.1f}%), "
        f"Unknown: {stats['unknown_cells']} ({stats['unknown_ratio']*100:.1f}%)"
    )

    # Save to a temporary isolated directory first to protect existing maps
    temp_dir = Path(tempfile.mkdtemp(prefix='tb4_map_save_'))
    temp_prefix = temp_dir / map_name
    cmd = [
        'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
        '-t', map_topic,
        '-f', str(temp_prefix),
        '--free', str(free_thresh),
        '--occ', str(occ_thresh),
        '--fmt', 'pgm',
        '--mode', 'trinary',
    ]

    print(f"[save_map] Invoking Nav2 map saver: {' '.join(cmd)}")
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        if proc.returncode != 0:
            raise RuntimeError(
                f"nav2_map_server map_saver_cli failed with exit code {proc.returncode}:\n{proc.stderr}\n{proc.stdout}"
            )
        temp_yaml = temp_dir / f'{map_name}.yaml'
        temp_pgm = temp_dir / f'{map_name}.pgm'
        verified_info = transactional_map_replace(temp_yaml, temp_pgm, target_yaml, target_pgm)
        print(f"[save_map] Map files written transactionally to:")
        print(f"  - YAML: {target_yaml}")
        print(f"  - PGM:  {target_pgm}")
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)

    verified_info.update(stats)
    verified_info['target_yaml'] = str(target_yaml)
    verified_info['target_pgm'] = str(target_pgm)
    return verified_info


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Safely and atomically save live SLAM map for TurtleBot 4.'
    )
    parser.add_argument(
        '--output-dir', default=None,
        help='Target output directory (default: package source maps/)'
    )
    parser.add_argument(
        '--map-name', default='map',
        help='Base map filename (produces <map-name>.yaml and <map-name>.pgm)'
    )
    parser.add_argument(
        '--map-topic', default='/map',
        help='OccupancyGrid topic name (default: /map)'
    )
    parser.add_argument(
        '--timeout', type=bounded_timeout, default=20.0,
        help='Operation timeout in seconds (1..120)'
    )
    parser.add_argument(
        '--free-thresh', type=float, default=DEFAULT_FREE_THRESH,
        help='Free cell threshold (default: 0.196; preserves unknown pixel 205)'
    )
    parser.add_argument(
        '--occ-thresh', type=float, default=DEFAULT_OCC_THRESH,
        help='Occupied cell threshold (default: 0.65)'
    )

    args = parser.parse_args(argv)

    rclpy.init()
    try:
        info = save_map_atomic(
            output_dir=args.output_dir,
            map_name=args.map_name,
            map_topic=args.map_topic,
            timeout=args.timeout,
            free_thresh=args.free_thresh,
            occ_thresh=args.occ_thresh,
        )
        print("\n[save_map] SUCCESS: Bản đồ đã được lưu và kiểm tra thành công!")
    except Exception as exc:
        print(f"\n[save_map] ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
