"""Deterministic tabletop slot selection from one camera-derived snapshot."""

from __future__ import annotations

from dataclasses import dataclass
from math import floor, hypot, isfinite
from typing import Callable, Mapping

from ur3_perception_llm_control.perception_state import (
    DEFAULT_MAX_AGE_SEC, PerceptionSnapshot, PerceptionStateError,
    PerceptionStateStatus, Rectangle, WorkcellGeometry, classify_location,
)
from ur3_perception_llm_control.workcell_scene import iter_models
from ur3_perception_llm_control.world_state import BLOCKS, TABLE, ZONES


@dataclass(frozen=True)
class TemporaryPlacement:
    object_name: str
    candidate_id: int
    x: float
    y: float
    z: float
    minimum_clearance_m: float


@dataclass(frozen=True)
class CandidateStatistics:
    generated: int = 0
    rejected_table: int = 0
    rejected_zone: int = 0
    rejected_cube: int = 0
    rejected_fixed: int = 0
    geometric_valid: int = 0
    moveit_infeasible: int = 0
    moveit_feasible: int = 0


class TemporaryPositionError(ValueError):
    def __init__(self, code: str, message: str, statistics: CandidateStatistics | None = None):
        self.code = code
        self.statistics = statistics
        super().__init__(f"{code}: {message}")


class TemporaryPositionPlanner:
    """Generate a table-anchored grid, then try safe slots in ranked order.

    Ranking is shortest XY transfer first, then largest minimum geometric
    clearance, then grid index. The first MoveIt-feasible candidate wins.
    """

    MARGINS = (
        "grid_spacing", "table_edge_clearance", "zone_clearance",
        "cube_clearance", "fixed_obstacle_clearance",
    )

    def __init__(self, scene: Mapping[str, object], geometry: WorkcellGeometry,
                 settings: Mapping[str, object]) -> None:
        self.geometry = geometry
        self.margins = {}
        for key in self.MARGINS:
            try:
                value = float(settings[key])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"temporary_position.{key} must be numeric") from exc
            if not isfinite(value) or value < 0 or (key == "grid_spacing" and value == 0):
                raise ValueError(f"temporary_position.{key} must be finite and positive/nonnegative")
            self.margins[key] = value
        models = {model.name: model for model in iter_models(scene)}
        table = models["manipulation_table"]
        self.table_top_z = table.pose[2] + table.size[2] / 2
        if (tuple(table.pose[:2]) != geometry.table.center_xy
                or tuple(table.size[:2]) != geometry.table.size_xy):
            raise ValueError("table geometry disagrees with symbolic workcell")
        self.cube_sizes = {name: models[name].size for name in BLOCKS}
        if any(tuple(self.cube_sizes[name][:2]) != geometry.cube_sizes[name] for name in BLOCKS):
            raise ValueError("cube geometry disagrees with symbolic workcell")
        self.fixed = tuple(
            Rectangle((model.pose[0], model.pose[1]), (model.size[0], model.size[1]))
            for model in models.values()
            if model.static and model.collision and model.name != table.name
        )
        if any(any(abs(angle) > 1e-9 for angle in model.pose[3:])
               for model in models.values() if model.static and model.collision):
            raise ValueError("fixed obstacle footprint requires axis-aligned geometry")

    def _validated_snapshot(self, snapshot: PerceptionSnapshot, now_sec: float) -> None:
        if not isinstance(snapshot, PerceptionSnapshot):
            raise TemporaryPositionError("PERCEPTION_INVALID", "PerceptionSnapshot required")
        if (set(snapshot.object_world_xy) != set(BLOCKS)
                or set(snapshot.object_locations) != set(BLOCKS)
                or set(snapshot.zone_occupancy) != set(ZONES)
                or snapshot.held_object is not None):
            raise TemporaryPositionError("PERCEPTION_INVALID", "complete static five-cube snapshot required")
        try:
            snapshot.require_fresh(now_sec, DEFAULT_MAX_AGE_SEC)
            occupancy = {zone: None for zone in ZONES}
            for name in BLOCKS:
                location = classify_location(name, snapshot.object_world_xy[name], self.geometry)
                if location != snapshot.object_locations[name]:
                    raise TemporaryPositionError("PERCEPTION_INVALID", f"{name} location disagrees with XY")
                if location != TABLE:
                    if occupancy[location] is not None:
                        raise TemporaryPositionError("PERCEPTION_INVALID", f"{location} has multiple cubes")
                    occupancy[location] = name
            if occupancy != dict(snapshot.zone_occupancy):
                raise TemporaryPositionError("PERCEPTION_INVALID", "zone occupancy disagrees with XY")
        except PerceptionStateError as exc:
            raise TemporaryPositionError(exc.status.value, str(exc)) from exc
        except (KeyError, TypeError, ValueError) as exc:
            if isinstance(exc, TemporaryPositionError):
                raise
            raise TemporaryPositionError("PERCEPTION_INVALID", str(exc)) from exc

    @staticmethod
    def _gap(left: Rectangle, right: Rectangle) -> float:
        dx = max(0.0, abs(left.center_xy[0] - right.center_xy[0])
                 - (left.size_xy[0] + right.size_xy[0]) / 2)
        dy = max(0.0, abs(left.center_xy[1] - right.center_xy[1])
                 - (left.size_xy[1] + right.size_xy[1]) / 2)
        return hypot(dx, dy)

    def candidate_rule(self, object_name: str, xy: tuple[float, float],
                       snapshot: PerceptionSnapshot) -> tuple[str | None, float]:
        """Return the first failed footprint rule and minimum raw XY clearance."""
        size = self.geometry.cube_sizes[object_name]
        candidate = Rectangle(xy, size)
        table = self.geometry.table
        edge = min(table.size_xy[axis] / 2 - abs(xy[axis] - table.center_xy[axis])
                   - size[axis] / 2 for axis in (0, 1))
        if edge + 1e-12 < self.margins["table_edge_clearance"]:
            return "table", edge
        zone_gaps = [self._gap(candidate, zone) for zone in self.geometry.zones.values()]
        if zone_gaps and min(zone_gaps) + 1e-12 < self.margins["zone_clearance"]:
            return "zone", min(zone_gaps)
        cube_gaps = [self._gap(candidate, Rectangle(snapshot.object_world_xy[name],
                                                    self.geometry.cube_sizes[name]))
                     for name in BLOCKS if name != object_name]
        if cube_gaps and min(cube_gaps) + 1e-12 < self.margins["cube_clearance"]:
            return "cube", min(cube_gaps)
        fixed_gaps = [self._gap(candidate, fixed) for fixed in self.fixed]
        if fixed_gaps and min(fixed_gaps) + 1e-12 < self.margins["fixed_obstacle_clearance"]:
            return "fixed", min(fixed_gaps)
        return None, min([edge, *zone_gaps, *cube_gaps, *fixed_gaps])

    def ranked_candidates(self, object_name: str, snapshot: PerceptionSnapshot,
                          now_sec: float) -> tuple[tuple[TemporaryPlacement, ...], CandidateStatistics]:
        self._validated_snapshot(snapshot, now_sec)
        if object_name not in BLOCKS:
            raise TemporaryPositionError("INVALID_OBJECT", object_name)
        table = self.geometry.table
        size = self.cube_sizes[object_name]
        spacing = self.margins["grid_spacing"]
        edge = self.margins["table_edge_clearance"]
        minimum = tuple(table.center_xy[axis] - table.size_xy[axis] / 2
                        + size[axis] / 2 + edge for axis in (0, 1))
        maximum = tuple(table.center_xy[axis] + table.size_xy[axis] / 2
                        - size[axis] / 2 - edge for axis in (0, 1))
        counts = tuple(max(0, floor((maximum[axis] - minimum[axis]) / spacing + 1e-9) + 1)
                       for axis in (0, 1))
        rejected = {"table": 0, "zone": 0, "cube": 0, "fixed": 0}
        candidates = []
        for iy in range(counts[1]):
            for ix in range(counts[0]):
                index = iy * counts[0] + ix
                xy = (minimum[0] + ix * spacing, minimum[1] + iy * spacing)
                reason, clearance = self.candidate_rule(object_name, xy, snapshot)
                if reason is not None:
                    rejected[reason] += 1
                    continue
                candidates.append(TemporaryPlacement(
                    object_name, index, xy[0], xy[1], self.table_top_z + size[2] / 2,
                    clearance,
                ))
        origin = snapshot.object_world_xy[object_name]
        candidates.sort(key=lambda candidate: (
            hypot(candidate.x - origin[0], candidate.y - origin[1]),
            -candidate.minimum_clearance_m, candidate.candidate_id,
        ))
        stats = CandidateStatistics(
            generated=counts[0] * counts[1], rejected_table=rejected["table"],
            rejected_zone=rejected["zone"], rejected_cube=rejected["cube"],
            rejected_fixed=rejected["fixed"], geometric_valid=len(candidates),
        )
        return tuple(candidates), stats

    def find_temporary_position(self, object_name: str, snapshot: PerceptionSnapshot,
                                now_sec: float,
                                is_feasible: Callable[[TemporaryPlacement], bool]
                                ) -> tuple[TemporaryPlacement, CandidateStatistics]:
        candidates, stats = self.ranked_candidates(object_name, snapshot, now_sec)
        for candidate in candidates:
            if is_feasible(candidate):
                return candidate, CandidateStatistics(
                    **{**stats.__dict__, "moveit_feasible": 1}
                )
            stats = CandidateStatistics(
                **{**stats.__dict__, "moveit_infeasible": stats.moveit_infeasible + 1}
            )
        raise TemporaryPositionError(
            "NO_TEMPORARY_POSITION", "no safe reachable tabletop slot", stats,
        )
