"""Offline tests only: synthetic speech/actions do not prove live acceptance."""

import copy
import hashlib
import math
from pathlib import Path
from types import SimpleNamespace
import threading

import numpy as np
from PIL import Image
import pytest
import yaml

from turtlebot4.map_segmentation import (
    WarehouseMap,
    DEFINITIONS,
    generate,
    load_regions,
    polygon_mask,
    validate_polygons,
    visualize,
)
from turtlebot4.speech_recognition_node import UtteranceBuffer
from turtlebot4.voice_command_parser import (
    VoiceCommandParser,
    TranscriptGate,
    normalize,
)
from turtlebot4.voice_navigation_node import (
    MissionExecutor,
    MissionStopped,
    VoiceNavigationNode,
    goal_pose,
    validate_live_goal,
)
from turtlebot4.semantic_map_node import build_markers
from builtin_interfaces.msg import Time
from nav2_msgs.msg import Costmap

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def grid():
    return WarehouseMap(ROOT / "maps/map.yaml")


@pytest.fixture(scope="module")
def config(grid):
    return generate(grid)


@pytest.mark.parametrize("ident", list("ABCDEFGHI"))
def test_all_region_goals_and_definitions(grid, config, ident):
    region = config["regions"][ord(ident) - ord("A")]
    definition = DEFINITIONS[ord(ident) - ord("A")]
    assert region["name"] == definition[1]
    assert region["aliases"] == definition[3]
    assert region["validation"] == "VALID"
    assert grid.validate_goal(region["approach_goal"]) >= 0.55
    assert region["nav2_path"] == "NOT TESTED"
    assert region["reachability"] == "GEOMETRIC_ONLY"
    mask = polygon_mask(grid.image.shape, region["polygon_pixels"])
    assert mask.any()


@pytest.mark.parametrize("yaw", [0.0, 0.42, -1.7, math.pi])
@pytest.mark.parametrize("pixel", [(0, 0), (600, 999), (280.3, 721.7)])
def test_pixel_center_roundtrip(grid, yaw, pixel):
    g = copy.copy(grid)
    g.origin = (-15.0, -25.1, yaw)
    assert g.world_to_pixel(*g.pixel_to_world(*pixel)) == pytest.approx(pixel)


def test_y_axis_and_centers(grid):
    assert grid.pixel_to_world(0, grid.height - 1) == pytest.approx((-14.975, -25.075))
    assert grid.pixel_to_world(0, 0)[1] > grid.pixel_to_world(0, 1)[1]


def test_map_occupancy_and_negate(tmp_path):
    image = np.array([[0, 255, 205], [255, 255, 0]], np.uint8)
    Image.fromarray(image).save(tmp_path / "test.pgm")
    meta = dict(
        image="test.pgm",
        resolution=0.1,
        origin=[1, 2, 0.3],
        negate=0,
        occupied_thresh=0.65,
        free_thresh=0.196,
    )
    path = tmp_path / "map.yaml"
    path.write_text(yaml.safe_dump(meta))
    grid = WarehouseMap(path)
    assert grid.occupied[0, 0] and grid.free[0, 1] and grid.unknown[0, 2]
    meta["negate"] = 1
    path.write_text(yaml.safe_dump(meta))
    grid = WarehouseMap(path)
    assert grid.free[0, 0] and grid.occupied[0, 1]


def test_reference_racks_and_geometry(grid, config):
    masks = {
        r["id"]: polygon_mask(grid.image.shape, r["polygon_pixels"])
        for r in config["regions"]
    }
    assert masks["B"][750, 120] and masks["C"][750, 260] and masks["D"][750, 420]
    assert masks["I"][260, 40] and masks["H"][60, 300] and masks["A"][970, 300]
    assert masks["G"][220, 300] and masks["F"][460, 300] and masks["E"][750, 540]
    assert np.max(sum(m.astype(int) for m in masks.values())) == 1
    duplicate = copy.deepcopy(config["regions"])
    duplicate[1]["polygon_pixels"] = duplicate[0]["polygon_pixels"]
    with pytest.raises(ValueError, match="overlap"):
        validate_polygons(grid, duplicate)


@pytest.mark.parametrize("point", [(0, 0), (120, 700), (-10, 50)])
def test_blocked_goals(grid, point):
    x, y = grid.pixel_to_world(*point)
    with pytest.raises(ValueError):
        grid.validate_goal(dict(x=x, y=y, yaw=0.0))


def test_unreachable_handling(grid):
    blocked = copy.copy(grid)
    blocked.safe = np.zeros_like(grid.safe)
    assert all(
        r["validation"] == "INVALID" and r["approach_goal"] is None
        for r in generate(blocked)["regions"]
    )


def test_stale_regions_and_original_hashes(tmp_path, config):
    path = tmp_path / "regions.yaml"
    bad = copy.deepcopy(config)
    bad["map_sha256"]["pgm"] = "bad"
    path.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError, match="Stale"):
        load_regions(path, ROOT / "maps/map.yaml")
    expected = yaml.safe_load((ROOT / "config/goals.yaml").read_text())["map_sha256"]
    assert config["map_sha256"] == expected
    load_regions(ROOT / "maps/regions.yaml", ROOT / "maps/map.yaml")


def test_deterministic_artifacts(tmp_path, grid, config):
    assert yaml.safe_dump(generate(grid)) == yaml.safe_dump(config)
    for name in ("one.png", "two.png"):
        visualize(grid, config, tmp_path / name)
    assert (tmp_path / "one.png").read_bytes() == (tmp_path / "two.png").read_bytes()
    assert (ROOT / "maps/segmented_map.png").read_bytes() == (
        tmp_path / "one.png"
    ).read_bytes()


@pytest.mark.parametrize("ident", list("ABCDEFGHI"))
def test_region_ids(ident):
    assert VoiceCommandParser().parse(f"Đi đến khu vực {ident}").regions == (ident,)


@pytest.mark.parametrize("definition", DEFINITIONS)
def test_all_aliases(definition):
    ident, name, _, aliases, _ = definition
    for alias in [name, *aliases]:
        for text in [alias, normalize(alias), alias.upper()]:
            assert VoiceCommandParser().parse("Đi đến " + text).regions == (ident,)


@pytest.mark.parametrize(
    "text,expected",
    [
        ("Đi đến khu cảm biến", ("B",)),
        ("Đi đến khu truyền động", ("C",)),
        ("Đi đến khu điều khiển", ("D",)),
        ("Đi đến khu trung tâm", ("F",)),
        ("Đi đến khu nguồn", ("G",)),
        ("Đi đến phòng họp", ("I",)),
        ("Đi đến sảnh phòng họp", ("H",)),
        ("Đi từ khu vực A đến khu vực B", ("A", "B")),
        ("Di chuyển từ khu cảm biến đến khu nguồn", ("B", "G")),
        ("Đi từ khu trung tâm đến phòng họp", ("F", "I")),
        ("Đi từ khu A qua khu F rồi đến khu G", ("A", "F", "G")),
        ("  ĐI ĐẾN,   KHU VỰC A!!! ", ("A",)),
        ("di den khu cam bien", ("B",)),
    ],
)
def test_required_phrases(text, expected):
    assert VoiceCommandParser().parse(text).regions == expected


@pytest.mark.parametrize(
    "text",
    [
        "Đi đến khu vực Z",
        "Đi đến phòng không tồn tại",
        "",
        "hello",
        "Đi đến khu A và khu B",
        "Đi đến khu A phòng họp",
        "Đi từ khu A đến khu Z",
        "Đi đến khu dê",
    ],
)
def test_unknown_ambiguous_rejected(text):
    command = VoiceCommandParser().parse(text)
    assert command.error and not command.regions


@pytest.mark.parametrize("text", ["Dừng lại", "DUNG LAI!", "Hủy nhiệm vụ"])
def test_cancel(text):
    assert VoiceCommandParser().parse(text).cancel


def test_duplicate_transcript():
    gate = TranscriptGate()
    assert gate.accept("Đi đến khu A", 1.0)
    assert not gate.accept("di den khu a", 2.0)
    assert gate.accept("di den khu a", 6.0)


def test_vad_one_utterance_and_bounded_noise():
    buffer = UtteranceBuffer()
    results = []
    for value in [0.0] * 20 + [0.1] * 30 + [0.0] * 70:
        result = buffer.feed(np.full(320, value))
        if result is not None:
            results.append(result)
    assert len(results) == 1
    for _ in range(1000):
        assert buffer.feed(np.ones(320)) is None
        assert len(buffer.frames) <= buffer.max_frames
    for _ in range(40):
        buffer.feed(np.zeros(320))
    assert not buffer.dropping


def test_no_utterance_from_silence():
    b = UtteranceBuffer()
    for _ in range(1000):
        assert b.feed(np.zeros(320)) is None
    assert len(b.preroll) <= 10 and not b.frames


class Adapter:
    def __init__(self, fail=None):
        self.calls = []
        self.fail = fail

    def call(self, kind, ident):
        self.calls.append((kind, ident))
        if self.fail == kind:
            raise RuntimeError("failure")

    def validate(self, ident):
        self.call("validate", ident)

    def plan(self, ident):
        self.call("plan", ident)

    def navigate(self, ident):
        self.call("navigate", ident)


def test_sequential_route():
    adapter = Adapter()
    MissionExecutor(adapter).run(["A", "B"])
    assert adapter.calls == [
        (kind, ident)
        for ident in ("A", "B")
        for kind in ("validate", "plan", "navigate")
    ]


@pytest.mark.parametrize("failure", ["validate", "plan", "navigate"])
def test_no_second_goal_after_failure(failure):
    adapter = Adapter(failure)
    with pytest.raises(RuntimeError):
        MissionExecutor(adapter).run(["A", "B"])
    assert all(ident == "A" for _, ident in adapter.calls)


def test_map_pose_quaternion():
    pose = goal_pose(dict(x=1.0, y=2.0, yaw=math.pi), Time(sec=15))
    assert pose.header.frame_id == "map" and pose.header.stamp.sec == 15
    assert pose.pose.position.x == 1.0 and pose.pose.orientation.z == pytest.approx(1.0)


def costmap():
    grid = Costmap()
    grid.header.frame_id = "map"
    grid.metadata.size_x = grid.metadata.size_y = 40
    grid.metadata.resolution = 0.1
    grid.metadata.origin.orientation.w = 1.0
    grid.data = [0] * 1600
    return grid


@pytest.mark.parametrize("value", [253, 254, 255])
def test_live_costmap_footprint_rejection(value):
    grid = costmap()
    goal = dict(x=2.0, y=2.0, yaw=0.0)
    validate_live_goal(goal, grid)
    grid.data[20 * 40 + 21] = value
    with pytest.raises(ValueError, match="footprint blocked"):
        validate_live_goal(goal, grid)


def test_action_rejected_no_navigation():
    obj = SimpleNamespace(
        pending=None,
        goal_handle=None,
        probe=SimpleNamespace(action=None),
        check_cancel=lambda: None,
        feedback=lambda m: None,
    )
    client = SimpleNamespace(
        server_is_ready=lambda: True, send_goal_async=lambda *a, **k: object()
    )
    obj.resolve_pending = lambda: None
    with pytest.raises(RuntimeError, match="rejected"):
        VoiceNavigationNode.send(obj, client, object(), 1.0)


@pytest.mark.parametrize("status", [4, 5, 6])
def test_action_terminal_results(status):
    obj = SimpleNamespace(
        pending=None,
        goal_handle=None,
        probe=SimpleNamespace(action=None),
        check_cancel=lambda: None,
        feedback=lambda m: None,
    )
    obj.resolve_pending = lambda: setattr(
        obj, "goal_handle", SimpleNamespace(get_result_async=lambda: object())
    )
    obj.await_future = lambda *a, **k: SimpleNamespace(status=status, result="success")
    client = SimpleNamespace(
        server_is_ready=lambda: True, send_goal_async=lambda *a, **k: object()
    )
    if status == 4:
        assert VoiceNavigationNode.send(obj, client, object(), 1.0) == "success"
    else:
        with pytest.raises(RuntimeError, match="failed"):
            VoiceNavigationNode.send(obj, client, object(), 1.0)


def test_timeout_and_cancel():
    obj = SimpleNamespace(probe=None, check_cancel=lambda: None)
    with pytest.raises(TimeoutError):
        VoiceNavigationNode.await_future(obj, SimpleNamespace(done=lambda: False), -1.0)
    obj.cancel = threading.Event()
    obj.closing = threading.Event()
    obj.cancel.set()
    with pytest.raises(MissionStopped):
        VoiceNavigationNode.check_cancel(obj)


def test_localization_rejected_before_plan():
    class NotReady(Adapter):
        def validate(self, ident):
            raise RuntimeError("Localization not ready")

    adapter = NotReady()
    with pytest.raises(RuntimeError, match="Localization"):
        MissionExecutor(adapter).run(["A", "B"])
    assert adapter.calls == []


def test_markers_names_goals_selection(config):
    markers = build_markers(config, "A").markers
    assert len(markers) == 37
    assert len({m.id for m in markers}) == 37
    assert all(m.header.frame_id == "map" for m in markers)
    assert [m.text for m in markers if m.type == 9 and " — " in m.text] == [
        r["id"] + " — " + r["name"] for r in config["regions"]
    ]
    assert markers[0].scale.x > markers[4].scale.x


def test_no_cmd_vel_and_preserved_interfaces():
    source = (ROOT / "turtlebot4/voice_navigation_node.py").read_text()
    assert "create_publisher(Twist" not in source
    assert "NavigateToPose.Goal" in source and "ComputePathToPose.Goal" in source
    setup = (ROOT / "setup.py").read_text()
    for executable in [
        "start_slam",
        "teleop",
        "save_map",
        "start_localization",
        "set_initial_pose",
        "start_navigation",
        "prepare_navigation",
        "waypoint_navigator",
        "yolo_detector",
        "gesture_recognition_node",
        "robot_behavior_node",
    ]:
        assert f"'{executable} = " in setup
    for name in ["simulation", "turtlebot4", "assignment5", "assignment6"]:
        assert (ROOT / f"launch/{name}.launch.py").is_file()


def test_real_node_worker_lifecycle():
    """Construct real ROS nodes: catches reserved names and executor startup faults."""
    import rclpy
    import time

    rclpy.init()
    node = None
    try:
        node = VoiceNavigationNode()
        assert node.worker.is_alive()
        assert node.probe.get_name() != node.get_name()
        assert node.probe.action._action_name == "/navigate_to_pose"
    finally:
        if node:
            node.stop()
            node.destroy_node()
        rclpy.shutdown()


def test_pending_acceptance_is_canceled_before_release():
    calls = []
    handle = SimpleNamespace(
        accepted=True,
        cancel_goal_async=lambda: calls.append("cancel") or "cancel",
        get_result_async=lambda: calls.append("result") or "result",
    )
    obj = SimpleNamespace(pending="send", goal_handle=None)
    obj.await_future = lambda f, *a, **k: (
        handle if f == "send" else SimpleNamespace(status=5)
    )
    obj.resolve_pending = lambda: VoiceNavigationNode.resolve_pending(obj)
    VoiceNavigationNode.cancel_active(obj)
    assert calls == ["cancel", "result"]
    assert obj.pending is None and obj.goal_handle is None


def test_recognized_invalid_duplicate_busy_and_cancel():
    import queue
    from std_msgs.msg import String

    events = []
    obj = SimpleNamespace(
        parser=VoiceCommandParser(),
        gate=TranscriptGate(),
        cancel=threading.Event(),
        closing=threading.Event(),
        lock=threading.Lock(),
        busy=False,
        jobs=queue.Queue(maxsize=1),
        current="",
        command="",
        ids=(),
        status=lambda state, **values: events.append((state, values)),
    )
    VoiceNavigationNode.recognized(obj, String(data="Đi đến khu vực Z"))
    assert obj.jobs.empty() and events[-1][0] == "FAILED"
    VoiceNavigationNode.recognized(obj, String(data="Đi từ khu A đến khu B"))
    assert obj.jobs.get_nowait() == ("A", "B") and obj.busy
    VoiceNavigationNode.recognized(obj, String(data="Đi đến khu C"))
    assert obj.jobs.empty()
    obj.busy = False
    VoiceNavigationNode.recognized(obj, String(data="Đi từ khu A đến khu B"))
    assert obj.jobs.empty()
    obj.busy = True
    VoiceNavigationNode.recognized(obj, String(data="Dừng lại"))
    assert obj.cancel.is_set() and obj.jobs.empty()


def test_real_plan_goal_generation(monkeypatch, config):
    from nav2_msgs.action import ComputePathToPose
    import turtlebot4.voice_navigation_node as module

    captured = []
    planner = object()
    probe = SimpleNamespace(
        planner=planner,
        messages={
            "clock": SimpleNamespace(clock=Time(sec=20)),
            "map": object(),
            "global": object(),
        },
    )
    obj = SimpleNamespace(
        probe=probe,
        regions={r["id"]: r for r in config["regions"]},
        status=lambda *a, **k: None,
    )
    obj.send = lambda client, goal, timeout: captured.append(
        (client, goal, timeout)
    ) or SimpleNamespace(path=object())
    monkeypatch.setattr(module, "validate_path", lambda *args: None)
    VoiceNavigationNode.plan(obj, "B")
    client, goal, timeout = captured[0]
    assert client is planner and isinstance(goal, ComputePathToPose.Goal)
    assert goal.planner_id == "GridBased" and not goal.use_start
    assert goal.goal.header.frame_id == "map"
    assert goal.goal.pose.position.x == config["regions"][1]["approach_goal"]["x"]


def test_live_map_mismatch_rejected(grid):
    from nav_msgs.msg import OccupancyGrid
    from turtlebot4.voice_navigation_node import validate_loaded_map

    msg = OccupancyGrid()
    msg.info.width, msg.info.height = grid.width, grid.height
    values = np.flipud(
        np.where(grid.free, 0, np.where(grid.occupied, 100, -1))
    ).reshape(-1)
    msg.data = values.astype(np.int8).tolist()
    validate_loaded_map(grid, msg)
    msg.data[0] = 0 if msg.data[0] != 0 else 100
    with pytest.raises(ValueError, match="differs"):
        validate_loaded_map(grid, msg)


def test_reproducible_vietnamese_rviz_texture(tmp_path, config):
    import xml.etree.ElementTree as ET
    from turtlebot4.map_segmentation import write_rviz_legend

    write_rviz_legend(config, tmp_path)
    for name in ("semantic_legend.png", "semantic_legend.dae"):
        assert (tmp_path / name).read_bytes() == (ROOT / "maps" / name).read_bytes()
    ET.parse(tmp_path / "semantic_legend.dae")
    assert Image.open(tmp_path / "semantic_legend.png").size == (650, 850)
    assert (
        build_markers(config).markers[-1].mesh_resource
        == "package://turtlebot4/maps/semantic_legend.dae"
    )
