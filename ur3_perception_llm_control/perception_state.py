"""Camera-derived, immutable five-cube symbolic state for a static observed scene."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from pathlib import Path
from types import MappingProxyType
from typing import Mapping

from ur3_perception_llm_control.cube_detector import ObservedBlock
from ur3_perception_llm_control.workcell_scene import load_scene
from ur3_perception_llm_control.world_state import BLOCKS, TABLE, ZONES


DEFAULT_MAX_AGE_SEC = 1.0
ZONE_CLEARANCE_M = 0.003  # Exceeds the measured M4 maximum XY error (2.561 mm).


class PerceptionStateStatus(str, Enum):
    INCOMPLETE = "PERCEPTION_INCOMPLETE"
    INVALID = "PERCEPTION_INVALID"
    AMBIGUOUS = "PERCEPTION_AMBIGUOUS"
    OCCUPANCY_CONFLICT = "PERCEPTION_OCCUPANCY_CONFLICT"
    STALE = "PERCEPTION_STALE"


class PerceptionStateError(ValueError):
    def __init__(self, status: PerceptionStateStatus, message: str) -> None:
        self.status = status
        super().__init__(f"{status.value}: {message}")


@dataclass(frozen=True)
class Rectangle:
    center_xy: tuple[float, float]
    size_xy: tuple[float, float]

    @classmethod
    def from_entry(cls, entry: object, label: str) -> "Rectangle":
        if not isinstance(entry, dict):
            raise PerceptionStateError(PerceptionStateStatus.INVALID, f"{label} geometry missing")
        pose, size = entry.get("pose"), entry.get("size")
        if not isinstance(pose, dict) or not isinstance(size, dict):
            raise PerceptionStateError(PerceptionStateStatus.INVALID, f"{label} pose/size missing")
        try:
            values = tuple(float(pose[key]) for key in ("x", "y", "roll", "pitch", "yaw"))
            dimensions = tuple(float(size[key]) for key in ("x", "y"))
        except (KeyError, ValueError, TypeError) as exc:
            raise PerceptionStateError(PerceptionStateStatus.INVALID, f"{label} pose/size invalid") from exc
        if (not all(isfinite(value) for value in (*values, *dimensions))
                or any(abs(value) > 1e-9 for value in values[2:])
                or any(value <= 0 for value in dimensions)):
            raise PerceptionStateError(PerceptionStateStatus.INVALID, f"{label} rectangle invalid")
        return cls((values[0], values[1]), dimensions)


@dataclass(frozen=True)
class WorkcellGeometry:
    table: Rectangle
    zones: Mapping[str, Rectangle]
    cube_sizes: Mapping[str, tuple[float, float]]
    clearance_m: float = ZONE_CLEARANCE_M

    @classmethod
    def from_scene_file(cls, path: str | Path, clearance_m: float = ZONE_CLEARANCE_M) -> "WorkcellGeometry":
        # Fixed zone/table geometry and cube dimensions only. Spawn pose fields
        # under scene['objects'] are never used for runtime localization.
        scene = load_scene(str(path))
        if not isfinite(clearance_m) or clearance_m < 0:
            raise PerceptionStateError(PerceptionStateStatus.INVALID, "clearance must be finite and nonnegative")
        table = Rectangle.from_entry(scene["table"], "table")
        zones = {name: Rectangle.from_entry(scene["zones"][name], name) for name in sorted(ZONES)}
        sizes = {}
        for name in BLOCKS:
            size = scene["objects"][name]["size"]
            xy = float(size["x"]), float(size["y"])
            if not all(isfinite(value) and value > 0 for value in xy):
                raise PerceptionStateError(PerceptionStateStatus.INVALID, f"{name} size invalid")
            sizes[name] = xy
        for zone_name, zone in zones.items():
            if not any(all(zone.size_xy[axis] > size[axis] + 2 * clearance_m for axis in (0, 1))
                       for size in sizes.values()):
                raise PerceptionStateError(PerceptionStateStatus.INVALID, f"{zone_name} cannot contain a cube")
        return cls(table, MappingProxyType(zones), MappingProxyType(sizes), clearance_m)


def classify_location(name: str, xy: tuple[float, float], geometry: WorkcellGeometry) -> str:
    """Require full footprint plus clearance for occupancy; reject partial overlap."""
    if name not in geometry.cube_sizes or not isinstance(xy, (tuple, list)) or len(xy) != 2:
        raise PerceptionStateError(PerceptionStateStatus.INVALID, f"{name} has invalid XY or dimensions")
    try:
        xy = float(xy[0]), float(xy[1])
    except (TypeError, ValueError) as exc:
        raise PerceptionStateError(PerceptionStateStatus.INVALID, f"{name} has invalid XY") from exc
    if not all(isfinite(value) for value in xy):
        raise PerceptionStateError(PerceptionStateStatus.INVALID, f"{name} has non-finite XY")
    cube_size = geometry.cube_sizes[name]
    for axis in (0, 1):
        if abs(xy[axis] - geometry.table.center_xy[axis]) + cube_size[axis] / 2 > geometry.table.size_xy[axis] / 2:
            raise PerceptionStateError(PerceptionStateStatus.INVALID, f"{name} footprint leaves the table")
    valid = []
    overlaps = []
    for zone_name, zone in geometry.zones.items():
        offsets = [abs(xy[axis] - zone.center_xy[axis]) for axis in (0, 1)]
        fit_limits = [(zone.size_xy[axis] - cube_size[axis]) / 2 - geometry.clearance_m for axis in (0, 1)]
        overlap_limits = [(zone.size_xy[axis] + cube_size[axis]) / 2 for axis in (0, 1)]
        if all(offsets[axis] <= fit_limits[axis] for axis in (0, 1)):
            valid.append(zone_name)
        if all(offsets[axis] < overlap_limits[axis] for axis in (0, 1)):
            overlaps.append(zone_name)
    if len(valid) == 1 and overlaps == valid:
        return valid[0]
    if valid or overlaps:
        raise PerceptionStateError(PerceptionStateStatus.AMBIGUOUS,
                                   f"{name} overlaps zone boundary or multiple zones")
    return TABLE


@dataclass(frozen=True)
class PerceptionSnapshot:
    """One complete M4 frame classified against fixed workcell geometry."""

    object_locations: Mapping[str, str]
    object_world_xy: Mapping[str, tuple[float, float]]
    zone_occupancy: Mapping[str, str | None]
    observation_timestamp_sec: float
    held_object: None = None

    @classmethod
    def from_detections(cls, detections: Mapping[str, ObservedBlock],
                        geometry: WorkcellGeometry) -> "PerceptionSnapshot":
        if set(detections) != set(BLOCKS):
            raise PerceptionStateError(PerceptionStateStatus.INCOMPLETE,
                                       "one coherent detection of each of the five blocks is required")
        locations = {}
        world_xy = {}
        occupancy: dict[str, str | None] = {zone: None for zone in sorted(ZONES)}
        timestamps = set()
        for name in BLOCKS:
            observed = detections[name]
            if observed.name != name or observed.timestamp_sec is None:
                raise PerceptionStateError(PerceptionStateStatus.INVALID, f"{name} identity/timestamp invalid")
            stamp = float(observed.timestamp_sec)
            if not isfinite(stamp) or stamp < 0:
                raise PerceptionStateError(PerceptionStateStatus.INVALID, f"{name} timestamp invalid")
            timestamps.add(stamp)
            xy = observed.world_xy
            location = classify_location(name, xy, geometry)
            locations[name] = location
            world_xy[name] = tuple(float(value) for value in xy)
            if location != TABLE:
                if occupancy[location] is not None:
                    raise PerceptionStateError(PerceptionStateStatus.OCCUPANCY_CONFLICT,
                                               f"{name} and {occupancy[location]} occupy {location}")
                occupancy[location] = name
        if len(timestamps) != 1:
            raise PerceptionStateError(PerceptionStateStatus.INVALID,
                                       "detections must come from one camera frame")
        return cls(MappingProxyType(locations), MappingProxyType(world_xy),
                   MappingProxyType(occupancy), timestamps.pop())

    def is_fresh(self, now_sec: float, max_age_sec: float = DEFAULT_MAX_AGE_SEC) -> bool:
        if not isfinite(now_sec) or not isfinite(max_age_sec) or max_age_sec <= 0:
            raise ValueError("time and maximum age must be finite; age must be positive")
        age = now_sec - self.observation_timestamp_sec
        return 0 <= age <= max_age_sec

    def require_fresh(self, now_sec: float, max_age_sec: float = DEFAULT_MAX_AGE_SEC) -> None:
        if not self.is_fresh(now_sec, max_age_sec):
            raise PerceptionStateError(PerceptionStateStatus.STALE, "camera snapshot is stale or from the future")
