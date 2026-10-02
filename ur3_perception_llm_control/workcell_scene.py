"""Load the canonical workcell YAML and generate simple box-model SDF."""

from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import Dict, Iterator, Mapping, Optional, Tuple
from xml.sax.saxutils import escape

import yaml
from ur3_perception_llm_control.world_state import BLOCKS


EXPECTED_OBJECTS = BLOCKS
EXPECTED_ZONES = ("zone_a", "zone_b", "zone_c")
POSE_FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")
VECTOR_FIELDS = ("x", "y", "z")
COLOR_FIELDS = ("r", "g", "b", "a")


@dataclass(frozen=True)
class BoxModel:
    """Validated description of one Gazebo box entity."""

    name: str
    pose: Tuple[float, float, float, float, float, float]
    size: Tuple[float, float, float]
    color: Tuple[float, float, float, float]
    static: bool
    collision: bool
    mass: Optional[float] = None


@dataclass(frozen=True)
class CameraConfig:
    name: str
    frame_id: str
    pose: Tuple[float, float, float, float, float, float]
    width: int
    height: int
    horizontal_fov: float
    update_rate: float
    image_topic: str
    camera_info_topic: str


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value


def _numeric_fields(
    value: object, fields: Tuple[str, ...], label: str
) -> Tuple[float, ...]:
    data = _mapping(value, label)
    missing = [field for field in fields if field not in data]
    if missing:
        raise ValueError(f"{label} is missing: {', '.join(missing)}")
    try:
        numbers = tuple(float(data[field]) for field in fields)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} fields must be numeric") from exc
    if not all(isfinite(number) for number in numbers):
        raise ValueError(f"{label} fields must be finite")
    return numbers


def _box_model(
    value: object,
    label: str,
    *,
    static: bool,
    collision: bool,
    mass_required: bool,
) -> BoxModel:
    data = _mapping(value, label)
    name = data.get("name")
    if not isinstance(name, str) or not name:
        raise ValueError(f"{label}.name must be a non-empty string")

    pose = _numeric_fields(data.get("pose"), POSE_FIELDS, f"{label}.pose")
    size = _numeric_fields(data.get("size"), VECTOR_FIELDS, f"{label}.size")
    color = _numeric_fields(data.get("color"), COLOR_FIELDS, f"{label}.color")
    if any(component <= 0.0 for component in size):
        raise ValueError(f"{label}.size fields must be positive")
    if any(component < 0.0 or component > 1.0 for component in color):
        raise ValueError(f"{label}.color fields must be between 0 and 1")

    mass: Optional[float] = None
    if mass_required:
        try:
            mass = float(data["mass"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"{label}.mass must be numeric") from exc
        if mass <= 0.0:
            raise ValueError(f"{label}.mass must be positive")
        if not isfinite(mass):
            raise ValueError(f"{label}.mass must be finite")

    return BoxModel(name, pose, size, color, static, collision, mass)


def camera_config(scene: Mapping[str, object]) -> CameraConfig:
    """Validate the fixed RGB sensor specified by the canonical scene."""
    data = _mapping(scene.get("camera"), "camera")
    name = data.get("name")
    frame_id = data.get("frame_id")
    if not isinstance(name, str) or not name or not isinstance(frame_id, str) or not frame_id:
        raise ValueError("camera name and frame_id must be non-empty strings")
    pose = _numeric_fields(data.get("pose"), POSE_FIELDS, "camera.pose")
    width, height = data.get("width"), data.get("height")
    if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
        raise ValueError("camera image dimensions must be positive integers")
    try:
        horizontal_fov = float(data["horizontal_fov"])
        update_rate = float(data["update_rate"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("camera field of view and update rate must be numeric") from exc
    if not (isfinite(horizontal_fov) and 0.0 < horizontal_fov < 3.141592653589793):
        raise ValueError("camera horizontal_fov must be between 0 and pi")
    if not (isfinite(update_rate) and update_rate > 0.0):
        raise ValueError("camera update_rate must be finite and positive")
    image_topic = data.get("image_topic")
    info_topic = data.get("camera_info_topic")
    if not all(isinstance(topic, str) and topic.startswith("/") for topic in (image_topic, info_topic)):
        raise ValueError("camera topics must be absolute names")
    if image_topic == info_topic:
        raise ValueError("camera image and info topics must differ")
    return CameraConfig(name, frame_id, pose, width, height, horizontal_fov,
                        update_rate, image_topic, info_topic)


def load_scene(path: str) -> Dict[str, object]:
    """Load and validate the M2.5 scene schema from ``path``."""
    with Path(path).open("r", encoding="utf-8") as stream:
        scene = yaml.safe_load(stream)
    root = _mapping(scene, "scene")

    if root.get("schema_version") != 2:
        raise ValueError("scene.schema_version must be 2")

    camera = camera_config(root)

    robot = _mapping(root.get("robot"), "robot")
    if robot.get("ur_type") not in ("ur3", "ur3e"):
        raise ValueError("robot.ur_type must be ur3 or ur3e")
    if robot.get("world_frame") != "world":
        raise ValueError("robot.world_frame must be world")
    mount_pose = _numeric_fields(
        robot.get("mount_pose"), POSE_FIELDS, "robot.mount_pose"
    )

    objects = _mapping(root.get("objects"), "objects")
    zones = _mapping(root.get("zones"), "zones")
    if set(objects) != set(EXPECTED_OBJECTS):
        raise ValueError(f"objects must contain exactly: {', '.join(EXPECTED_OBJECTS)}")
    if set(zones) != set(EXPECTED_ZONES):
        raise ValueError(f"zones must contain exactly: {', '.join(EXPECTED_ZONES)}")

    models = list(iter_models(root))
    names = [model.name for model in models]
    expected_names = [
        "robot_pedestal",
        "manipulation_table",
        *EXPECTED_OBJECTS,
        *EXPECTED_ZONES,
    ]
    if names != expected_names:
        raise ValueError(
            "entity names must be exactly: " + ", ".join(expected_names)
        )
    if camera.name in names:
        raise ValueError("camera name must be distinct from workcell models")

    pedestal = models[0]
    pedestal_top = pedestal.pose[2] + pedestal.size[2] / 2.0
    table = models[1]
    table_top = table.pose[2] + table.size[2] / 2.0
    if any(abs(mount_pose[index] - pedestal.pose[index]) > 1e-9 for index in (0, 1)):
        raise ValueError("robot mount XY must be centered on the pedestal")
    if abs(mount_pose[2] - pedestal_top) > 1e-9:
        raise ValueError("robot mount Z must equal the pedestal top")
    if abs(mount_pose[2] - table_top) > 1e-9:
        raise ValueError("robot mount Z must equal the tabletop height")

    return dict(root)


def iter_models(scene: Mapping[str, object]) -> Iterator[BoxModel]:
    """Yield support structures, cubes, and zones in deterministic order."""
    yield _box_model(
        scene.get("pedestal"),
        "pedestal",
        static=True,
        collision=True,
        mass_required=False,
    )

    yield _box_model(
        scene.get("table"),
        "table",
        static=True,
        collision=True,
        mass_required=False,
    )

    objects = _mapping(scene.get("objects"), "objects")
    for key in EXPECTED_OBJECTS:
        yield _box_model(
            objects.get(key),
            f"objects.{key}",
            static=False,
            collision=True,
            mass_required=True,
        )

    zones = _mapping(scene.get("zones"), "zones")
    for key in EXPECTED_ZONES:
        yield _box_model(
            zones.get(key),
            f"zones.{key}",
            static=True,
            collision=False,
            mass_required=False,
        )


def box_sdf(model: BoxModel) -> str:
    """Create SDF for a box, including inertia only for dynamic models."""
    size_text = " ".join(f"{value:.9g}" for value in model.size)
    color_text = " ".join(f"{value:.9g}" for value in model.color)
    collision_xml = ""
    if model.collision:
        collision_xml = f"""
      <collision name="collision">
        <geometry><box><size>{size_text}</size></box></geometry>
        <surface>
          <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>
        </surface>
      </collision>"""

    inertial_xml = ""
    if not model.static:
        assert model.mass is not None
        x_size, y_size, z_size = model.size
        ixx = model.mass * (y_size**2 + z_size**2) / 12.0
        iyy = model.mass * (x_size**2 + z_size**2) / 12.0
        izz = model.mass * (x_size**2 + y_size**2) / 12.0
        inertial_xml = f"""
      <inertial>
        <mass>{model.mass:.9g}</mass>
        <inertia>
          <ixx>{ixx:.9g}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy:.9g}</iyy><iyz>0</iyz><izz>{izz:.9g}</izz>
        </inertia>
      </inertial>"""

    return f"""<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{escape(model.name)}">
    <static>{str(model.static).lower()}</static>
    <allow_auto_disable>true</allow_auto_disable>
    <link name="link">{inertial_xml}{collision_xml}
      <visual name="visual">
        <geometry><box><size>{size_text}</size></box></geometry>
        <material>
          <ambient>{color_text}</ambient>
          <diffuse>{color_text}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def camera_sdf(camera: CameraConfig) -> str:
    """Create one static Gazebo Fortress RGB camera model without depth data."""
    return f"""<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{escape(camera.name)}">
    <static>true</static>
    <link name="camera_link">
      <sensor name="rgb" type="camera">
        <always_on>true</always_on>
        <update_rate>{camera.update_rate:.9g}</update_rate>
        <topic>{escape(camera.image_topic)}</topic>
        <camera_info_topic>{escape(camera.camera_info_topic)}</camera_info_topic>
        <frame_id>{escape(camera.frame_id)}</frame_id>
        <camera>
          <horizontal_fov>{camera.horizontal_fov:.9g}</horizontal_fov>
          <image>
            <width>{camera.width}</width><height>{camera.height}</height>
            <format>R8G8B8</format>
          </image>
          <clip><near>0.05</near><far>5.0</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>"""
