"""Synchronize MoveIt WORLD cube boxes from one fresh camera snapshot."""

from __future__ import annotations

from dataclasses import dataclass
from math import dist, isclose, isfinite
from types import MappingProxyType
from typing import Mapping

from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

from ur3_perception_llm_control.perception_state import (
    DEFAULT_MAX_AGE_SEC, PerceptionSnapshot, WorkcellGeometry, classify_location,
)
from ur3_perception_llm_control.planning_scene import (
    PlanningSceneManager, collision_object, object_color, scene_matches,
)
from ur3_perception_llm_control.workcell_scene import BoxModel, iter_models
from ur3_perception_llm_control.world_state import BLOCKS, TABLE, ZONES


SYNC_TOLERANCE_M = 0.001


class SceneSyncError(ValueError):
    """Snapshot rejected or authoritative Planning Scene did not match it."""


@dataclass(frozen=True)
class SceneSyncReport:
    requested_xyz: Mapping[str, tuple[float, float, float]]
    authoritative_xyz: Mapping[str, tuple[float, float, float]]
    sync_errors_m: Mapping[str, float]
    attached_ids: frozenset[str]
    max_sync_error_m: float
    service_accepted: bool


class PerceptionPlanningSceneSynchronizer:
    """Apply one five-block diff while preserving static and attached objects.

    RGB supplies XY only. Fixed support geometry supplies center Z and box
    dimensions. Identity orientation is deterministic for equal-edge cubes;
    no camera orientation estimate is claimed.
    """

    def __init__(self, manager: PlanningSceneManager, scene: dict[str, object],
                 geometry: WorkcellGeometry) -> None:
        self.manager = manager
        self.geometry = geometry
        models = {model.name: model for model in iter_models(scene)}
        self.frame_id = str(scene["robot"]["world_frame"])
        self.cubes = {name: models[name] for name in BLOCKS}
        self.static = {name: model for name, model in models.items()
                       if model.static and model.collision}
        self.table_top_z = models["manipulation_table"].pose[2] + models["manipulation_table"].size[2] / 2
        if set(self.static) != {"robot_pedestal", "manipulation_table"}:
            raise SceneSyncError("unexpected static collision geometry")
        for name, model in self.cubes.items():
            if model.static or not model.collision or tuple(model.size[:2]) != geometry.cube_sizes[name]:
                raise SceneSyncError(f"{name} geometry disagrees with symbolic workcell")
            if not all(isclose(size, model.size[0], abs_tol=1e-9) for size in model.size):
                raise SceneSyncError(f"{name} is not an equal-edge cube")

    def _validate_snapshot(self, snapshot: PerceptionSnapshot, now_sec: float,
                           max_age_sec: float) -> None:
        if not isinstance(snapshot, PerceptionSnapshot):
            raise SceneSyncError("a PerceptionSnapshot is required")
        if (set(snapshot.object_locations) != set(BLOCKS)
                or set(snapshot.object_world_xy) != set(BLOCKS)
                or set(snapshot.zone_occupancy) != set(ZONES)
                or snapshot.held_object is not None):
            raise SceneSyncError("snapshot is incomplete or has an unsupported held object")
        try:
            snapshot.require_fresh(now_sec, max_age_sec)
        except (ValueError, TypeError) as exc:
            raise SceneSyncError(str(exc)) from exc
        occupancy: dict[str, str | None] = {zone: None for zone in ZONES}
        for name in BLOCKS:
            try:
                location = classify_location(name, snapshot.object_world_xy[name], self.geometry)
            except (ValueError, TypeError) as exc:
                raise SceneSyncError(str(exc)) from exc
            if snapshot.object_locations[name] != location:
                raise SceneSyncError(f"{name} symbolic location disagrees with observed XY")
            if location != TABLE:
                if occupancy[location] is not None:
                    raise SceneSyncError(f"two blocks occupy {location}")
                occupancy[location] = name
        if dict(snapshot.zone_occupancy) != occupancy:
            raise SceneSyncError("snapshot zone occupancy is inconsistent")

    def requested_objects(self, snapshot: PerceptionSnapshot) -> dict[str, CollisionObject]:
        """Build boxes with camera XY, support-derived Z, and identity orientation."""
        result = {}
        for name in BLOCKS:
            model = self.cubes[name]
            x, y = snapshot.object_world_xy[name]
            # Zones are visual task regions. Every settled cube rests on the table.
            z = self.table_top_z + model.size[2] / 2
            # Construct a fresh model: the scene's movable spawn XYZ is never
            # copied into a runtime MoveIt request.
            observed = BoxModel(name, (float(x), float(y), z, 0.0, 0.0, 0.0),
                                model.size, model.color, False, True, model.mass)
            result[name] = collision_object(observed, self.frame_id)
        return result

    @staticmethod
    def _ids(scene: PlanningScene) -> tuple[list[str], list[str]]:
        return ([item.id for item in scene.world.collision_objects],
                [item.object.id for item in scene.robot_state.attached_collision_objects])

    def apply_snapshot(self, snapshot: PerceptionSnapshot, now_sec: float,
                       max_age_sec: float = DEFAULT_MAX_AGE_SEC) -> SceneSyncReport:
        # Complete, fresh, geometrically consistent input is required before
        # either a read of MoveIt or a mutation.
        self._validate_snapshot(snapshot, now_sec, max_age_sec)
        requested = self.requested_objects(snapshot)
        before = self.manager.get()
        if before is None:
            raise SceneSyncError("authoritative Planning Scene unavailable")
        world_ids, attached_list = self._ids(before)
        attached = frozenset(attached_list)
        if len(attached_list) != len(attached):
            raise SceneSyncError("duplicate attached IDs in authoritative scene")
        if not attached.issubset(BLOCKS):
            raise SceneSyncError("unexpected non-cube attachment in authoritative scene")
        if any(name in world_ids for name in attached):
            raise SceneSyncError("authoritative scene already has WORLD plus ATTACHED object")
        if not set(self.static).issubset(world_ids):
            raise SceneSyncError("static table or pedestal missing from authoritative scene")
        diff = PlanningScene()
        diff.is_diff = True
        diff.robot_state.is_diff = True
        diff.world.collision_objects = [requested[name] for name in BLOCKS if name not in attached]
        diff.object_colors = [object_color(self.cubes[name]) for name in BLOCKS if name not in attached]
        accepted = self.manager.apply_scene_diff(diff)
        # Humble may report false after applying a diff. The authoritative
        # query, not the service boolean, determines the result.
        return self.verify_snapshot(snapshot, attached_ids=attached, service_accepted=accepted)

    def verify_snapshot(self, snapshot: PerceptionSnapshot, *,
                        attached_ids: frozenset[str] | None = None,
                        service_accepted: bool = True) -> SceneSyncReport:
        requested = self.requested_objects(snapshot)
        actual = self.manager.get()
        if actual is None:
            raise SceneSyncError("authoritative Planning Scene unavailable after update")
        world_ids, attached_list = self._ids(actual)
        attached = frozenset(attached_list) if attached_ids is None else attached_ids
        if len(attached_list) != len(set(attached_list)) or set(attached_list) != set(attached):
            raise SceneSyncError("attached-object state changed during synchronization")
        if not attached.issubset(BLOCKS):
            raise SceneSyncError("unexpected non-cube attachment in authoritative scene")
        if any(name in world_ids for name in attached):
            raise SceneSyncError("WORLD plus ATTACHED duplicate detected")
        if any(name in world_ids or name in attached for name in ZONES):
            raise SceneSyncError("a visual-only zone became collision geometry")
        if any(world_ids.count(name) != 1 for name in self.static):
            raise SceneSyncError("static collision object missing or duplicated")
        static_actual = PlanningScene()
        static_expected = PlanningScene()
        static_actual.world.collision_objects = [item for item in actual.world.collision_objects if item.id in self.static]
        static_expected.world.collision_objects = [collision_object(model, self.frame_id) for model in self.static.values()]
        static_ok, static_detail = scene_matches(static_actual, static_expected)
        if not static_ok:
            raise SceneSyncError(f"static collision geometry changed: {static_detail}")
        actual_by_id = {item.id: item for item in actual.world.collision_objects}
        requested_xyz = {}
        authoritative_xyz = {}
        errors = {}
        for name in BLOCKS:
            wanted = requested[name]
            target = wanted.pose.position
            requested_xyz[name] = (target.x, target.y, target.z)
            if name in attached:
                if world_ids.count(name) != 0:
                    raise SceneSyncError(f"{name} is both WORLD and ATTACHED")
                continue
            if world_ids.count(name) != 1:
                raise SceneSyncError(f"{name}: expected exactly one WORLD collision object")
            found = actual_by_id[name]
            if (found.header.frame_id != self.frame_id or len(found.primitives) != 1
                    or found.primitives[0].type != SolidPrimitive.BOX
                    or len(found.primitive_poses) != 1):
                raise SceneSyncError(f"{name}: frame or box geometry is invalid")
            if any(not isclose(a, b, abs_tol=1e-7, rel_tol=0.0)
                   for a, b in zip(found.primitives[0].dimensions, wanted.primitives[0].dimensions)):
                raise SceneSyncError(f"{name}: box dimensions differ")
            if len(found.primitives[0].dimensions) != 3:
                raise SceneSyncError(f"{name}: box dimensions are incomplete")
            pose = found.pose
            xyz = (pose.position.x, pose.position.y, pose.position.z)
            if not all(isfinite(value) for value in xyz):
                raise SceneSyncError(f"{name}: non-finite authoritative pose")
            error = dist(xyz, requested_xyz[name])
            if error > SYNC_TOLERANCE_M:
                raise SceneSyncError(f"{name}: requested/authoritative pose error {error*1000:.3f} mm")
            if any(not isclose(a, b, abs_tol=1e-6, rel_tol=0.0)
                   for a, b in zip((pose.orientation.x, pose.orientation.y,
                                    pose.orientation.z, pose.orientation.w), (0.0, 0.0, 0.0, 1.0))):
                raise SceneSyncError(f"{name}: orientation differs from deterministic identity")
            authoritative_xyz[name] = xyz
            errors[name] = error
        return SceneSyncReport(MappingProxyType(requested_xyz), MappingProxyType(authoritative_xyz),
                               MappingProxyType(errors), attached, max(errors.values(), default=0.0),
                               service_accepted)
