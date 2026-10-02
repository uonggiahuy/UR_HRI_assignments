"""Static M2 RGB sensor and visual-only zone checks."""

from __future__ import annotations

from math import isfinite
from pathlib import Path
import unittest
from xml.etree import ElementTree

from ur3_perception_llm_control.planning_scene import collision_models
from ur3_perception_llm_control.workcell_scene import camera_config, camera_sdf, iter_models, load_scene
from ur3_perception_llm_control.zone_markers import TOPIC, zone_marker_array


ROOT = Path(__file__).resolve().parents[1]


class CameraZoneTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.scene = load_scene(str(ROOT / "config" / "scene.yaml"))

    def test_fixed_rgb_camera_configuration(self) -> None:
        camera = camera_config(self.scene)
        self.assertEqual(camera.name, "overhead_rgb_camera")
        self.assertEqual(camera.frame_id, "overhead_rgb_camera/camera_link/rgb")
        self.assertEqual((camera.width, camera.height), (640, 480))
        self.assertEqual((camera.image_topic, camera.camera_info_topic),
                         ("/camera/image_raw", "/camera/camera_info"))
        self.assertTrue(all(isfinite(value) for value in camera.pose))
        self.assertGreater(camera.pose[2], 0.3)
        self.assertGreater(camera.horizontal_fov, 0.0)
        self.assertGreater(camera.update_rate, 0.0)

        sdf = ElementTree.fromstring(camera_sdf(camera))
        self.assertEqual(sdf.findtext("./model/static"), "true")
        sensors = sdf.findall("./model/link/sensor")
        self.assertEqual(len(sensors), 1)
        self.assertEqual(sensors[0].attrib["type"], "camera")
        self.assertEqual(sensors[0].findtext("./camera/image/format"), "R8G8B8")
        self.assertEqual(sensors[0].findtext("topic"), camera.image_topic)
        self.assertEqual(sensors[0].findtext("camera_info_topic"), camera.camera_info_topic)
        world = ElementTree.parse(ROOT / "worlds" / "rgb_workcell.sdf")
        systems = {plugin.attrib.get("filename") for plugin in world.findall("./world/plugin")}
        self.assertIn("ignition-gazebo-sensors-system", systems)

    def test_markers_use_all_canonical_zone_values(self) -> None:
        zones = {model.name: model for model in iter_models(self.scene) if model.name.startswith("zone_")}
        markers = zone_marker_array(self.scene).markers
        self.assertEqual(len(markers), 3)
        self.assertEqual([marker.id for marker in markers], [0, 1, 2])
        self.assertEqual({marker.ns for marker in markers}, {"workcell_zones"})
        self.assertEqual(TOPIC, "/workcell/zone_markers")
        for name, marker in zip(("zone_a", "zone_b", "zone_c"), markers):
            model = zones[name]
            self.assertEqual(marker.header.frame_id, "world")
            self.assertEqual((marker.pose.position.x, marker.pose.position.y, marker.pose.position.z), model.pose[:3])
            self.assertEqual((marker.scale.x, marker.scale.y, marker.scale.z), model.size)
            for actual, expected in zip(
                (marker.color.r, marker.color.g, marker.color.b, marker.color.a), model.color
            ):
                self.assertAlmostEqual(actual, expected, places=6)

    def test_zones_remain_outside_moveit_collision_geometry(self) -> None:
        collision_names = {model.name for model in collision_models(self.scene)}
        self.assertFalse({"zone_a", "zone_b", "zone_c"} & collision_names)
        self.assertEqual(len(collision_names), 7)

    def test_assignment_rviz_config_enables_zone_markers(self) -> None:
        text = (ROOT / "rviz" / "workcell.rviz").read_text(encoding="utf-8")
        self.assertIn("rviz_default_plugins/MarkerArray", text)
        self.assertIn("Value: /workcell/zone_markers", text)


if __name__ == "__main__":
    unittest.main()
