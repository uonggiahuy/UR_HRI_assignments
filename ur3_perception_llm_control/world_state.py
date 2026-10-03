"""Authoritative task-level state for the configured M8 workcell."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import hypot
from pathlib import Path
from typing import Any

import yaml


LEGACY_STUDENT_OBJECTS = frozenset(("red_cube", "yellow_cube", "blue_cube"))
BLOCKS = ("red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube")
ZONES = frozenset(("zone_a", "zone_b", "zone_c"))
TABLE = "table"
PLAN_LAYOUT_TOLERANCE_M = 0.010


@dataclass
class WorldState:
    """Task state updated only by :class:`SkillExecutor` after a success."""

    held_object: str | None
    object_locations: dict[str, str]
    zone_occupancy: dict[str, str | None]
    initial_object_poses: dict[str, dict[str, float]] = field(default_factory=dict)
    revision: int = 0
    observed_xy: dict[str, tuple[float, float]] = field(default_factory=dict)
    perception_timestamp_sec: float | None = None

    @classmethod
    def from_perception_snapshot(cls, snapshot, now_sec: float, geometry) -> "WorldState":
        """Build an Assignment 03 state from one fresh, consistent RGB frame."""
        from ur3_perception_llm_control.perception_state import classify_location

        snapshot.require_fresh(now_sec)
        if (set(snapshot.object_world_xy) != set(BLOCKS)
                or set(snapshot.object_locations) != set(BLOCKS)
                or set(snapshot.zone_occupancy) != ZONES
                or snapshot.held_object is not None):
            raise ValueError("complete unheld five-cube snapshot required")
        occupancy = {zone: None for zone in ZONES}
        for name in BLOCKS:
            location = classify_location(name, snapshot.object_world_xy[name], geometry)
            if location != snapshot.object_locations[name]:
                raise ValueError(f"{name} camera location disagrees with XY")
            if location in ZONES:
                if occupancy[location] is not None:
                    raise ValueError(f"multiple cubes occupy {location}")
                occupancy[location] = name
        if occupancy != dict(snapshot.zone_occupancy):
            raise ValueError("camera zone occupancy is inconsistent")
        return cls(None, dict(snapshot.object_locations), occupancy,
                   observed_xy=dict(snapshot.object_world_xy),
                   perception_timestamp_sec=snapshot.observation_timestamp_sec)

    def matches_perception_snapshot(self, snapshot, now_sec: float,
                                    tolerance_m: float = PLAN_LAYOUT_TOLERANCE_M) -> bool:
        """Guard a validated plan against a newer, changed camera layout."""
        try:
            snapshot.require_fresh(now_sec)
            return (self.perception_timestamp_sec is not None
                    and snapshot.observation_timestamp_sec >= self.perception_timestamp_sec
                    and dict(snapshot.object_locations) == self.object_locations
                    and dict(snapshot.zone_occupancy) == self.zone_occupancy
                    and set(snapshot.object_world_xy) == set(self.observed_xy)
                    and all(hypot(snapshot.object_world_xy[name][0] - self.observed_xy[name][0],
                                  snapshot.object_world_xy[name][1] - self.observed_xy[name][1])
                            <= tolerance_m for name in BLOCKS))
        except (AttributeError, KeyError, TypeError, ValueError):
            return False

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
            observed_xy=dict(self.observed_xy),
            perception_timestamp_sec=self.perception_timestamp_sec,
        )

    def signature(self) -> tuple:
        """Immutable validation binding, including edits that skip revision APIs."""
        return (self.held_object,
                tuple(sorted(self.object_locations.items())),
                tuple(sorted(self.zone_occupancy.items())),
                tuple(sorted(self.observed_xy.items())),
                self.perception_timestamp_sec)

    def record_pick_success(self, object_name: str) -> None:
        """Apply the state transition caused by a successful ``pick`` skill."""
        previous = self.object_locations[object_name]
        if previous in ZONES:
            self.zone_occupancy[previous] = None
        self.held_object = object_name
        self.object_locations[object_name] = "held"
        self.revision += 1

    def record_temp_place_success(self, object_name: str) -> None:
        """A released temporary cube is a camera-classified table cube."""
        self.held_object = None
        self.object_locations[object_name] = TABLE
        self.revision += 1

    def record_place_success(self, object_name: str, zone_name: str) -> None:
        """Apply the state transition caused by a successful ``place`` skill."""
        self.held_object = None
        self.object_locations[object_name] = zone_name
        self.zone_occupancy[zone_name] = object_name
        self.revision += 1
