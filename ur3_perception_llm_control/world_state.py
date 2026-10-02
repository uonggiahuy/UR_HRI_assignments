"""Authoritative task-level state for the configured M8 workcell."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


LEGACY_STUDENT_OBJECTS = frozenset(("red_cube", "yellow_cube", "blue_cube"))
BLOCKS = ("red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube")
ZONES = frozenset(("zone_a", "zone_b", "zone_c"))
TABLE = "table"


@dataclass
class WorldState:
    """Task state updated only by :class:`SkillExecutor` after a success."""

    held_object: str | None
    object_locations: dict[str, str]
    zone_occupancy: dict[str, str | None]
    initial_object_poses: dict[str, dict[str, float]] = field(default_factory=dict)
    revision: int = 0

    @classmethod
    def from_scene_file(cls, scene_file: str | Path) -> "WorldState":
        """Create initial table state from the canonical ``scene.yaml`` objects."""
        with Path(scene_file).open(encoding="utf-8") as stream:
            scene = yaml.safe_load(stream)
        if not isinstance(scene, dict) or not isinstance(scene.get("objects"), dict):
            raise ValueError("scene.yaml must contain an objects mapping")

        objects = scene["objects"]
        if set(objects) != set(BLOCKS):
            raise ValueError("scene.yaml object names do not match the five-block workcell")
        poses: dict[str, dict[str, float]] = {}
        for object_name in BLOCKS:
            entry = objects[object_name]
            pose = entry.get("pose") if isinstance(entry, dict) else None
            if not isinstance(pose, dict):
                raise ValueError(f"scene.yaml object {object_name} has no pose")
            poses[object_name] = {key: float(value) for key, value in pose.items()}
        return cls(
            held_object=None,
            object_locations={object_name: TABLE for object_name in BLOCKS},
            zone_occupancy={zone_name: None for zone_name in ZONES},
            initial_object_poses=poses,
        )

    def copy(self) -> "WorldState":
        """Return an isolated snapshot for semantic validation simulation."""
        return WorldState(
            held_object=self.held_object,
            object_locations=dict(self.object_locations),
            zone_occupancy=dict(self.zone_occupancy),
            initial_object_poses={name: dict(pose) for name, pose in self.initial_object_poses.items()},
            revision=self.revision,
        )

    def record_pick_success(self, object_name: str) -> None:
        """Apply the state transition caused by a successful ``pick`` skill."""
        self.held_object = object_name
        self.object_locations[object_name] = "held"
        self.revision += 1

    def record_place_success(self, object_name: str, zone_name: str) -> None:
        """Apply the state transition caused by a successful ``place`` skill."""
        self.held_object = None
        self.object_locations[object_name] = zone_name
        self.zone_occupancy[zone_name] = object_name
        self.revision += 1
