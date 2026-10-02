"""M4 configuration, segmentation, fail-closed, and known-plane tests."""

from __future__ import annotations

from copy import deepcopy
from math import hypot
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np
import yaml

from ur3_perception_llm_control.cube_detector import (
    CUBE_NAMES, CubeDetector, DetectionError, PerceptionConfig, evaluate_stability,
)
from ur3_perception_llm_control.planar_mapper import Calibration, CubeTopMapper, PlanarMapper


ROOT = Path(__file__).resolve().parents[1]


class CubeDetectorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = PerceptionConfig.from_file(ROOT / "config/perception.yaml")
        cls.table = PlanarMapper(Calibration.from_file(ROOT / "config/camera_calibration.yaml"))
        cls.top = CubeTopMapper(cls.table, cls.config.camera_xyz, cls.config.cube_top_z_m)
        cls.detector = CubeDetector(cls.config, cls.top)

    def synthetic(self):
        hsv = np.zeros((480, 640, 3), np.uint8)
        centers = [(345, 301), (345, 240), (345, 178), (278, 301), (278, 178)]
        values = [(0, 191, 214), (27, 192, 219), (113, 191, 214),
                  (66, 185, 197), (146, 164, 193)]
        for (u, v), color in zip(centers, values):
            cv2.rectangle(hsv, (u-11, v-11), (u+11, v+11), color, -1)
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    def test_config_has_five_classes_and_red_wrap(self):
        self.assertEqual(set(self.config.hsv_ranges), set(CUBE_NAMES))
        self.assertTrue(any(bounds[0] == 0 for bounds in self.config.hsv_ranges["red_cube"]))
        self.assertTrue(any(bounds[1] == 179 for bounds in self.config.hsv_ranges["red_cube"]))
        self.assertEqual(self.config.cube_top_z_m, 0.345)

    def test_config_rejects_missing_class_and_bad_bounds(self):
        raw = yaml.safe_load((ROOT / "config/perception.yaml").read_text())
        for edit in (lambda d: d["colors"].pop("blue_cube"),
                     lambda d: d["colors"]["red_cube"]["hsv_ranges"][0].__setitem__(1, 180)):
            data = deepcopy(raw)
            edit(data)
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "perception.yaml"
                path.write_text(yaml.safe_dump(data))
                with self.assertRaises(ValueError):
                    PerceptionConfig.from_file(path)

    def test_synthetic_five_cubes_and_stability(self):
        image = self.synthetic()
        detections = self.detector.detect(image, 1.0)
        self.assertEqual(set(detections), set(CUBE_NAMES))
        self.assertTrue(all(300 <= item.contour_area_px <= 800 for item in detections.values()))
        samples = [self.detector.detect(image, float(index)) for index in range(self.config.frames)]
        state = evaluate_stability(samples, self.config)
        self.assertTrue(state.stable)
        self.assertLess(state.max_spread_m, 1e-12)

    def test_green_and_purple_on_similarly_colored_zones(self):
        hsv = cv2.cvtColor(self.synthetic(), cv2.COLOR_BGR2HSV)
        # The zone colors and cube top colors are measured M2 RGB values.
        cv2.rectangle(hsv, (369, 161), (409, 201), (63, 128, 183), -1)
        cv2.rectangle(hsv, (369, 220), (409, 260), (153, 145, 176), -1)
        hsv[290:313, 267:290] = 0  # Move green from the original layout.
        hsv[167:190, 267:290] = 0  # Move purple from the original layout.
        cv2.rectangle(hsv, (381, 167), (403, 189), (66, 185, 197), -1)
        cv2.rectangle(hsv, (381, 229), (403, 251), (146, 164, 193), -1)
        found = self.detector.detect(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR))
        self.assertEqual(set(found), set(CUBE_NAMES))
        self.assertLess(hypot(found["green_cube"].world_xy[0]-0.12,
                              found["green_cube"].world_xy[1]-0.24), 0.01)
        self.assertLess(hypot(found["purple_cube"].world_xy[0],
                              found["purple_cube"].world_xy[1]-0.24), 0.01)

    def test_unstable_measurements_are_rejected(self):
        image = self.synthetic()
        samples = [self.detector.detect(image) for _ in range(self.config.frames)]
        moved = image.copy()
        moved[290:313, 334:357] = 0
        moved[290:313, 345:368] = image[295, 340]
        samples[-1] = self.detector.detect(moved)
        self.assertFalse(evaluate_stability(samples, self.config).stable)

    def test_missing_duplicate_and_invalid_area_fail_closed(self):
        image = self.synthetic()
        image[290:313, 334:357] = 0  # Remove red.
        with self.assertRaisesRegex(DetectionError, "red_cube"):
            self.detector.detect(image)
        image = self.synthetic()
        cv2.rectangle(image, (300, 325), (322, 347), (54, 54, 214), -1)
        # Use exactly the red top-face BGR value to create an ambiguous second red.
        image[325:348, 300:323] = image[295, 340]
        with self.assertRaisesRegex(DetectionError, "red_cube"):
            self.detector.detect(image)
        image = self.synthetic()
        image[290:313, 334:357] = 0
        image[300:304, 340:344] = cv2.cvtColor(
            np.array([[[0, 191, 214]]], dtype=np.uint8), cv2.COLOR_HSV2BGR)[0, 0]
        with self.assertRaisesRegex(DetectionError, "red_cube"):
            self.detector.detect(image)

    def test_cube_plane_round_trip_and_tabletop_unchanged(self):
        for x, y in ((-0.12, 0.33), (0.0, 0.33), (0.12, 0.46)):
            u, v = self.top.world_xy_to_pixel(x, y)
            actual = self.top.pixel_to_world_xy(u, v)
            self.assertLess(hypot(actual[0]-x, actual[1]-y), 1e-6)
            table_xy = self.table.pixel_to_world_xy(u, v)
            self.assertGreater(hypot(table_xy[0]-x, table_xy[1]-y), 0.001)
        self.assertLess(hypot(*(np.subtract(self.table.pixel_to_world_xy(320, 240),
                                            (0.0, 0.38)))), 0.00001)
        with self.assertRaises(ValueError):
            CubeTopMapper(self.table, self.config.camera_xyz, 1.25)
        with self.assertRaises(ValueError):
            self.top.pixel_to_world_xy(0, 0)

    def test_no_depth_or_pose_dependency(self):
        import inspect
        import ur3_perception_llm_control.cube_detector as module
        source = inspect.getsource(module)
        for forbidden in ("gazebo_msgs", "gazebo", "depth", "scene.yaml", "model_states"):
            self.assertNotIn(forbidden, source.lower())


if __name__ == "__main__":
    unittest.main()
