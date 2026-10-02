"""Pure RGB/HSV localization of the five visible cube top faces."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite
from pathlib import Path
from typing import Mapping

import cv2
import numpy as np
import yaml

from ur3_perception_llm_control.planar_mapper import CubeTopMapper


CUBE_NAMES = ("red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube")


class DetectionError(ValueError):
    """A frame lacks one unambiguous, geometrically valid detection per cube."""


@dataclass(frozen=True)
class ObservedBlock:
    name: str
    pixel_center: tuple[float, float]
    world_xy: tuple[float, float]
    contour_area_px: int
    quality: float
    timestamp_sec: float | None
    bbox_xywh: tuple[int, int, int, int]


@dataclass(frozen=True)
class PerceptionConfig:
    camera_xyz: tuple[float, float, float]
    cube_top_z_m: float
    frames: int
    max_xy_spread_m: float
    hsv_ranges: Mapping[str, tuple[tuple[int, int, int, int, int, int], ...]]
    min_area_px: int
    max_area_px: int
    min_fill_ratio: float
    min_aspect_ratio: float
    max_aspect_ratio: float
    morphology_kernel_px: int

    @classmethod
    def from_file(cls, path: str | Path) -> "PerceptionConfig":
        with Path(path).open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict) or data.get("schema_version") != 1:
            raise ValueError("perception schema_version must be 1")
        camera = data.get("camera_center_world_xyz_m")
        if not isinstance(camera, list) or len(camera) != 3:
            raise ValueError("camera center must contain XYZ")
        camera_xyz = tuple(float(x) for x in camera)
        top_z = float(data.get("cube_top_z_m"))
        stability, blob, colors = data.get("stability"), data.get("blob"), data.get("colors")
        if not all(isfinite(x) for x in (*camera_xyz, top_z)):
            raise ValueError("camera and top plane must be finite")
        if not isinstance(stability, dict) or not isinstance(blob, dict) or not isinstance(colors, dict):
            raise ValueError("stability, blob and colors must be mappings")
        if set(colors) != set(CUBE_NAMES):
            raise ValueError("exactly five cube colors must be configured")
        ranges = {}
        for name in CUBE_NAMES:
            item = colors[name]
            if not isinstance(item, dict) or not isinstance(item.get("hsv_ranges"), list) or not item["hsv_ranges"]:
                raise ValueError(f"{name} needs HSV ranges")
            parsed = []
            for bounds in item["hsv_ranges"]:
                if not isinstance(bounds, list) or len(bounds) != 6 or any(type(n) is not int for n in bounds):
                    raise ValueError(f"{name} has invalid HSV range shape")
                h0, h1, s0, s1, v0, v1 = bounds
                if not (0 <= h0 <= h1 <= 179 and 0 <= s0 <= s1 <= 255 and 0 <= v0 <= v1 <= 255):
                    raise ValueError(f"{name} has invalid HSV bounds")
                parsed.append(tuple(bounds))
            ranges[name] = tuple(parsed)
        frames = blob_int(stability, "frames")
        kernel = blob_int(blob, "morphology_kernel_px")
        min_area, max_area = blob_int(blob, "min_area_px"), blob_int(blob, "max_area_px")
        spread = float(stability.get("max_xy_spread_m"))
        fill = float(blob.get("min_fill_ratio"))
        min_aspect, max_aspect = float(blob.get("min_aspect_ratio")), float(blob.get("max_aspect_ratio"))
        if (frames < 2 or kernel < 1 or kernel % 2 != 1 or min_area < 1 or max_area < min_area
                or not all(isfinite(n) for n in (spread, fill, min_aspect, max_aspect))
                or spread <= 0 or not 0 < fill <= 1 or not 0 < min_aspect <= max_aspect):
            raise ValueError("invalid blob or stability limits")
        return cls(camera_xyz, top_z, frames, spread, ranges, min_area, max_area,
                   fill, min_aspect, max_aspect, kernel)


def blob_int(data: Mapping[str, object], key: str) -> int:
    value = data.get(key)
    if type(value) is not int:
        raise ValueError(f"{key} must be an integer")
    return value


class CubeDetector:
    def __init__(self, config: PerceptionConfig, mapper: CubeTopMapper) -> None:
        self.config = config
        self.mapper = mapper

    def detect(self, frame_bgr: np.ndarray, timestamp_sec: float | None = None) -> dict[str, ObservedBlock]:
        if (not isinstance(frame_bgr, np.ndarray) or frame_bgr.dtype != np.uint8
                or frame_bgr.ndim != 3 or frame_bgr.shape != (
                    self.mapper.tabletop.calibration.image_height,
                    self.mapper.tabletop.calibration.image_width, 3)):
            raise DetectionError("expected a calibrated uint8 BGR image")
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        result = {}
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.config.morphology_kernel_px,) * 2)
        for name in CUBE_NAMES:
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for h0, h1, s0, s1, v0, v1 in self.config.hsv_ranges[name]:
                mask |= cv2.inRange(hsv, (h0, s0, v0), (h1, s1, v1))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            count, _, stats, centers = cv2.connectedComponentsWithStats(mask)
            candidates = []
            for index in range(1, count):
                x, y, width, height, area = (int(n) for n in stats[index])
                if not self.config.min_area_px <= area <= self.config.max_area_px:
                    continue
                aspect = width / height
                fill = area / (width * height)
                if not (self.config.min_aspect_ratio <= aspect <= self.config.max_aspect_ratio
                        and fill >= self.config.min_fill_ratio):
                    continue
                u, v = (float(n) for n in centers[index])
                try:
                    world_xy = self.mapper.pixel_to_world_xy(u, v)
                except ValueError:
                    continue
                candidates.append(ObservedBlock(name, (u, v), world_xy, area, fill,
                                                timestamp_sec, (x, y, width, height)))
            if len(candidates) != 1:
                raise DetectionError(f"{name}: expected one valid top-face blob, found {len(candidates)}")
            result[name] = candidates[0]
        return result


@dataclass(frozen=True)
class StabilityResult:
    stable: bool
    max_spread_m: float
    mean_xy: Mapping[str, tuple[float, float]]


def evaluate_stability(samples: list[Mapping[str, ObservedBlock]], config: PerceptionConfig) -> StabilityResult:
    if len(samples) < config.frames or any(set(sample) != set(CUBE_NAMES) for sample in samples):
        raise DetectionError("insufficient complete frames for stability evaluation")
    means = {}
    max_spread = 0.0
    for name in CUBE_NAMES:
        points = np.asarray([sample[name].world_xy for sample in samples], dtype=float)
        mean = points.mean(axis=0)
        spread = max(hypot(*(point - mean)) for point in points)
        max_spread = max(max_spread, spread)
        means[name] = float(mean[0]), float(mean[1])
    return StabilityResult(max_spread <= config.max_xy_spread_m, max_spread, means)
