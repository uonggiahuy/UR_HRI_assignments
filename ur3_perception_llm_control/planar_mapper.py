"""Finite, bounded RGB-pixel <-> tabletop-XY homography for the fixed camera."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import Mapping

import cv2
import numpy as np
import yaml


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return value


def _number(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be finite numeric data")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be finite numeric data") from exc
    if not isfinite(result):
        raise ValueError(f"{name} must be finite numeric data")
    return result


def _points(value: object, coordinate_names: tuple[str, str], label: str,
            minimum: int = 4) -> tuple[tuple[str, float, float], ...]:
    if not isinstance(value, list) or len(value) < minimum:
        raise ValueError(f"{label} needs at least {minimum} points")
    result = []
    for index, item in enumerate(value):
        item = _mapping(item, f"{label}[{index}]")
        name = item.get("id")
        if not isinstance(name, str) or not name:
            raise ValueError(f"{label}[{index}].id must be non-empty")
        result.append((name, *(_number(item.get(key), f"{label}[{index}].{key}") for key in coordinate_names)))
    if len({item[0] for item in result}) != len(result):
        raise ValueError(f"{label} IDs must be unique")
    return tuple(result)


def _finite_vector(value: object, length: int, name: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{name} must contain {length} numbers")
    return tuple(_number(item, name) for item in value)


@dataclass(frozen=True)
class Calibration:
    plane_frame: str
    plane_z_m: float
    image_width: int
    image_height: int
    image_frame: str
    distortion_model: str
    distortion_coefficients: tuple[float, ...]
    k: tuple[float, ...]
    image_points: tuple[tuple[str, float, float], ...]
    world_points: tuple[tuple[str, float, float], ...]
    validation_pixels: tuple[tuple[str, float, float], ...]

    @classmethod
    def from_file(cls, path: str | Path) -> "Calibration":
        with Path(path).open(encoding="utf-8") as stream:
            root = _mapping(yaml.safe_load(stream), "calibration")
        if root.get("schema_version") != 1:
            raise ValueError("calibration schema_version must be 1")
        plane = _mapping(root.get("plane"), "plane")
        image = _mapping(root.get("image"), "image")
        pairs = _mapping(root.get("calibration"), "calibration points")
        validation = _mapping(root.get("validation"), "validation")
        frame = plane.get("frame_id")
        image_frame = image.get("frame_id")
        if not isinstance(frame, str) or not frame or not isinstance(image_frame, str) or not image_frame:
            raise ValueError("plane and image frame IDs must be non-empty")
        width, height = image.get("width_px"), image.get("height_px")
        if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
            raise ValueError("image dimensions must be positive integers")
        model = image.get("distortion_model")
        if model != "plumb_bob":
            raise ValueError("only the inspected plumb_bob CameraInfo model is supported")
        distortion = _finite_vector(image.get("distortion_coefficients"), 5, "distortion coefficients")
        if any(value != 0.0 for value in distortion):
            raise ValueError("nonzero distortion requires rectified pixels before this homography")
        k = _finite_vector(image.get("k"), 9, "K matrix")
        if k[0] <= 0 or k[4] <= 0 or abs(k[8] - 1.0) > 1e-9:
            raise ValueError("K matrix has invalid focal length or scale")
        image_points = _points(pairs.get("image_points_px"), ("u", "v"), "image_points_px")
        world_points = _points(pairs.get("world_points_xy_m"), ("x", "y"), "world_points_xy_m")
        if len(image_points) != len(world_points) or [p[0] for p in image_points] != [p[0] for p in world_points]:
            raise ValueError("image and world calibration IDs must match in order")
        validation_pixels = _points(validation.get("image_points_px"), ("u", "v"), "validation pixels", 1)
        for name, u, v in (*image_points, *validation_pixels):
            if not (0.0 <= u < width and 0.0 <= v < height):
                raise ValueError(f"{name} is outside the image")
        return cls(frame, _number(plane.get("z_m"), "plane.z_m"), width, height,
                   image_frame, model, distortion, k, image_points, world_points,
                   validation_pixels)


class PlanarMapper:
    """Map points inside the calibrated tabletop; reject extrapolation."""

    def __init__(self, calibration: Calibration) -> None:
        self.calibration = calibration
        pixels = np.asarray([(u, v) for _, u, v in calibration.image_points], dtype=np.float64)
        world = np.asarray([(x, y) for _, x, y in calibration.world_points], dtype=np.float64)
        self._pixel_hull = cv2.convexHull(pixels.astype(np.float32))
        self._world_hull = cv2.convexHull(world.astype(np.float32))
        if (cv2.contourArea(self._pixel_hull) <= 1.0 or
                cv2.contourArea(self._world_hull) <= 1e-8 or
                np.linalg.matrix_rank(pixels - pixels[0]) < 2 or
                np.linalg.matrix_rank(world - world[0]) < 2):
            raise ValueError("calibration correspondences are collinear or degenerate")
        homography, _ = cv2.findHomography(pixels, world, method=0)
        if homography is None or not np.isfinite(homography).all():
            raise ValueError("OpenCV could not fit a finite homography")
        if (abs(float(homography[2, 2])) <= 1e-12 or
                abs(float(np.linalg.det(homography))) < 1e-12 or
                np.linalg.cond(homography) > 1e12):
            raise ValueError("homography is singular or ill-conditioned")
        self.image_to_world_h = homography / homography[2, 2]
        inverse = np.linalg.inv(self.image_to_world_h)
        if abs(float(inverse[2, 2])) <= 1e-12 or not np.isfinite(inverse).all():
            raise ValueError("inverse homography is invalid")
        self.world_to_image_h = inverse / inverse[2, 2]
        for (_, u, v), (_, x, y) in zip(calibration.image_points, calibration.world_points):
            if np.hypot(*(np.subtract(self._project(self.image_to_world_h, u, v), (x, y)))) > 0.002:
                raise ValueError("calibration reprojection error exceeds 2 mm")

    @staticmethod
    def _project(matrix: np.ndarray, first: float, second: float) -> tuple[float, float]:
        homogeneous = matrix @ np.array((first, second, 1.0), dtype=np.float64)
        denominator = float(homogeneous[2])
        if not np.isfinite(homogeneous).all() or abs(denominator) <= 1e-12:
            raise ValueError("projection has an invalid homogeneous denominator")
        result = homogeneous[:2] / denominator
        if not np.isfinite(result).all():
            raise ValueError("projection is non-finite")
        return float(result[0]), float(result[1])

    @staticmethod
    def _inside(hull: np.ndarray, first: float, second: float, tolerance: float) -> bool:
        return cv2.pointPolygonTest(hull, (first, second), True) >= -tolerance

    def pixel_to_world_xy(self, u: float, v: float) -> tuple[float, float]:
        u, v = _number(u, "u"), _number(v, "v")
        if not (0 <= u < self.calibration.image_width and 0 <= v < self.calibration.image_height):
            raise ValueError("pixel is outside the image")
        if not self._inside(self._pixel_hull, u, v, 1e-3):
            raise ValueError("pixel is outside the calibrated tabletop")
        return self._project(self.image_to_world_h, u, v)

    def world_xy_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        x, y = _number(x, "x"), _number(y, "y")
        if not self._inside(self._world_hull, x, y, 1e-6):
            raise ValueError("world XY is outside the calibrated tabletop")
        return self._project(self.world_to_image_h, x, y)


class CubeTopMapper:
    """Intersect a tabletop-calibrated camera ray with a known cube-top plane.

    The M3 homography supplies the ray's tabletop intersection. Similar
    triangles about the fixed camera center give its intersection at top_z.
    No cube pose or depth measurement enters this mapping.
    """

    def __init__(self, tabletop: PlanarMapper, camera_xyz: tuple[float, float, float],
                 top_z_m: float) -> None:
        self.tabletop = tabletop
        self.camera_x, self.camera_y, self.camera_z = (
            _number(value, "camera coordinate") for value in camera_xyz
        )
        self.top_z_m = _number(top_z_m, "cube-top Z")
        table_z = tabletop.calibration.plane_z_m
        if not table_z < self.top_z_m < self.camera_z:
            raise ValueError("cube-top plane must lie strictly between table and camera")
        self._scale = (self.camera_z - self.top_z_m) / (self.camera_z - table_z)

    def pixel_to_world_xy(self, u: float, v: float) -> tuple[float, float]:
        table_x, table_y = self.tabletop.pixel_to_world_xy(u, v)
        x = self.camera_x + self._scale * (table_x - self.camera_x)
        y = self.camera_y + self._scale * (table_y - self.camera_y)
        self.tabletop.world_xy_to_pixel(x, y)  # Reject locations outside the table.
        return x, y

    def world_xy_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        x, y = _number(x, "x"), _number(y, "y")
        self.tabletop.world_xy_to_pixel(x, y)
        table_x = self.camera_x + (x - self.camera_x) / self._scale
        table_y = self.camera_y + (y - self.camera_y) / self._scale
        return self.tabletop.world_xy_to_pixel(table_x, table_y)
