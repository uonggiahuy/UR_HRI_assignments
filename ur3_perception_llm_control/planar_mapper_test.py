"""Static M3 tabletop homography, input validation, and orientation checks."""

from __future__ import annotations

from copy import deepcopy
from math import hypot
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

from ur3_perception_llm_control.planar_mapper import Calibration, PlanarMapper
from ur3_perception_llm_control.workcell_scene import load_scene


ROOT = Path(__file__).resolve().parents[1]
CALIBRATION_FILE = ROOT / "config" / "camera_calibration.yaml"
SCENE_FILE = ROOT / "config" / "scene.yaml"


class PlanarMapperTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.calibration = Calibration.from_file(CALIBRATION_FILE)
        cls.mapper = PlanarMapper(cls.calibration)
        cls.scene = load_scene(str(SCENE_FILE))

    def _modified_calibration(self, edit) -> Calibration:
        data = deepcopy(yaml.safe_load(CALIBRATION_FILE.read_text(encoding="utf-8")))
        edit(data)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration.yaml"
            path.write_text(yaml.safe_dump(data), encoding="utf-8")
            return Calibration.from_file(path)

    def test_four_table_corners_reproject_both_directions(self) -> None:
        self.assertEqual(len(self.calibration.image_points), 4)
        self.assertEqual(self.calibration.plane_frame, "world")
        self.assertEqual(self.calibration.plane_z_m, 0.3)
        for (name, u, v), (_, x, y) in zip(self.calibration.image_points,
                                           self.calibration.world_points):
            with self.subTest(name=name):
                found_x, found_y = self.mapper.pixel_to_world_xy(u, v)
                found_u, found_v = self.mapper.world_xy_to_pixel(x, y)
                self.assertLess(hypot(found_x - x, found_y - y), 0.00001)
                self.assertLess(hypot(found_u - u, found_v - v), 0.01)

    def test_interior_round_trip_and_fixed_zone_holdouts(self) -> None:
        for x, y in ((0.0, 0.38), (-0.20, 0.30), (0.20, 0.50)):
            with self.subTest(x=x, y=y):
                u, v = self.mapper.world_xy_to_pixel(x, y)
                mapped_x, mapped_y = self.mapper.pixel_to_world_xy(u, v)
                self.assertLess(hypot(mapped_x - x, mapped_y - y), 0.00001)
        for name, u, v in self.calibration.validation_pixels:
            pose = self.scene["zones"][name]["pose"]
            x, y = self.mapper.pixel_to_world_xy(u, v)
            self.assertLess(hypot(x - pose["x"], y - pose["y"]), 0.005)

    def test_image_axes_have_expected_world_orientation(self) -> None:
        x0, y0 = self.mapper.pixel_to_world_xy(320.0, 240.0)
        x_right, y_right = self.mapper.pixel_to_world_xy(330.0, 240.0)
        x_down, y_down = self.mapper.pixel_to_world_xy(320.0, 250.0)
        self.assertAlmostEqual(x_right, x0, places=5)
        self.assertLess(y_right, y0)
        self.assertLess(x_down, x0)
        self.assertAlmostEqual(y_down, y0, places=5)

    def test_invalid_inputs_and_extrapolation_fail_closed(self) -> None:
        for u, v in ((float("nan"), 240.0), (float("inf"), 240.0),
                     (-1.0, 240.0), (0.0, 0.0)):
            with self.subTest(u=u, v=v), self.assertRaises(ValueError):
                self.mapper.pixel_to_world_xy(u, v)
        for x, y in ((float("nan"), 0.38), (0.0, float("inf")), (1.0, 1.0)):
            with self.subTest(x=x, y=y), self.assertRaises(ValueError):
                self.mapper.world_xy_to_pixel(x, y)
        with self.assertRaises(ValueError):
            PlanarMapper._project(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0]],
                                           dtype=float), 320.0, 240.0)

    def test_malformed_or_collinear_calibration_is_rejected(self) -> None:
        edits = (
            lambda d: d["calibration"]["image_points_px"].pop(),
            lambda d: d["calibration"]["world_points_xy_m"][0].update(id="wrong"),
            lambda d: d["calibration"]["image_points_px"][0].update(u=float("nan")),
            lambda d: d["image"].update(distortion_coefficients=[0.1, 0, 0, 0, 0]),
        )
        for edit in edits:
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                self._modified_calibration(edit)

        world_collinear = self._modified_calibration(
            lambda d: [point.update(y=0.38) for point in d["calibration"]["world_points_xy_m"]]
        )
        with self.assertRaises(ValueError):
            PlanarMapper(world_collinear)
        image_collinear = self._modified_calibration(
            lambda d: [point.update(v=240.0) for point in d["calibration"]["image_points_px"]]
        )
        with self.assertRaises(ValueError):
            PlanarMapper(image_collinear)


if __name__ == "__main__":
    unittest.main()
