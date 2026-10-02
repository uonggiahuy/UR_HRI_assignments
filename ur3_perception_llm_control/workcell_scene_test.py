"""Static checks for the canonical five-block Gazebo workcell."""

from __future__ import annotations

from math import isfinite
from pathlib import Path
import unittest

from ur3_perception_llm_control.workcell_scene import box_sdf, iter_models, load_scene
from ur3_perception_llm_control.world_state import BLOCKS, LEGACY_STUDENT_OBJECTS, WorldState


SCENE_FILE = Path(__file__).resolve().parents[1] / "config" / "scene.yaml"


class WorkcellSceneTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.scene = load_scene(str(SCENE_FILE))
        cls.models = {model.name: model for model in iter_models(cls.scene)}

    def test_exactly_five_unique_dynamic_collidable_cubes(self) -> None:
        cubes = [model for model in iter_models(self.scene) if model.name in BLOCKS]
        self.assertEqual(len(cubes), 5)
        self.assertEqual({model.name for model in cubes}, set(BLOCKS))
        self.assertEqual(len({model.name for model in iter_models(self.scene)}), 10)
        self.assertTrue(all(not model.static and model.collision and model.mass == 0.08 for model in cubes))
        self.assertEqual({model.size for model in cubes}, {(0.045, 0.045, 0.045)})
        self.assertEqual(len({model.color for model in cubes}), 5)
        self.assertTrue(all("<static>false</static>" in box_sdf(model) for model in cubes))

    def test_cube_poses_are_finite_and_on_table(self) -> None:
        table = self.models["manipulation_table"]
        table_top = table.pose[2] + table.size[2] / 2
        for name in BLOCKS:
            with self.subTest(name=name):
                cube = self.models[name]
                self.assertTrue(all(isfinite(value) for value in cube.pose))
                self.assertLessEqual(abs(cube.pose[0] - table.pose[0]) + cube.size[0] / 2,
                                     table.size[0] / 2)
                self.assertLessEqual(abs(cube.pose[1] - table.pose[1]) + cube.size[1] / 2,
                                     table.size[1] / 2)
                self.assertGreaterEqual(cube.pose[2] - cube.size[2] / 2, table_top)
                self.assertLessEqual(cube.pose[2] - cube.size[2] / 2 - table_top, 0.002)

    def test_new_cubes_clear_all_cubes_and_zones(self) -> None:
        for new_name in ("green_cube", "purple_cube"):
            new = self.models[new_name]
            for other_name in BLOCKS:
                if new_name == other_name:
                    continue
                other = self.models[other_name]
                self.assertTrue(
                    abs(new.pose[0] - other.pose[0]) >= (new.size[0] + other.size[0]) / 2
                    or abs(new.pose[1] - other.pose[1]) >= (new.size[1] + other.size[1]) / 2,
                    f"{new_name} overlaps {other_name}",
                )
            for zone_name in ("zone_a", "zone_b", "zone_c"):
                zone = self.models[zone_name]
                self.assertTrue(
                    abs(new.pose[0] - zone.pose[0]) >= (new.size[0] + zone.size[0]) / 2
                    or abs(new.pose[1] - zone.pose[1]) >= (new.size[1] + zone.size[1]) / 2,
                    f"{new_name} starts inside {zone_name}",
                )

    def test_world_state_tracks_five_blocks_but_legacy_task_has_three(self) -> None:
        state = WorldState.from_scene_file(SCENE_FILE)
        self.assertEqual(set(state.object_locations), set(BLOCKS))
        self.assertEqual(set(state.initial_object_poses), set(BLOCKS))
        self.assertEqual(LEGACY_STUDENT_OBJECTS, {"red_cube", "yellow_cube", "blue_cube"})
        self.assertEqual(set(state.zone_occupancy), {"zone_a", "zone_b", "zone_c"})


if __name__ == "__main__":
    unittest.main()
