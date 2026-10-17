"""Goal visualization and packaged model dependencies use the real assets."""
from pathlib import Path

from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

from turtlebot4.goal_markers import make_markers
from turtlebot4.waypoint_navigator import load_goals

ROOT = Path(__file__).resolve().parents[1]


def test_goal_markers_preserve_all_configured_centers_and_order():
    config = load_goals(ROOT / 'config/goals.yaml', ROOT / 'maps/map.yaml')
    stamp = Time(sec=10)
    result = make_markers(config, stamp)
    assert len(result.markers) == 6
    assert len({m.id for m in result.markers}) == 6
    for index, goal in enumerate(config['goals']):
        sphere, label = result.markers[index * 2:index * 2 + 2]
        assert sphere.type == Marker.SPHERE and label.type == Marker.TEXT_VIEW_FACING
        for m in (sphere, label):
            assert m.header.frame_id == 'map' and m.header.stamp == stamp
            assert m.pose.position.x == goal['x']
            expected_y = goal['y'] + (.30 if m.type == Marker.TEXT_VIEW_FACING else 0.)
            assert m.pose.position.y == expected_y
            assert m.color.a == 1.
        assert label.text == f"Goal {index + 1}"


def test_all_obj_materials_and_referenced_textures_are_packaged():
    for obj in (ROOT / 'models').rglob('*.obj'):
        for line in obj.read_text().splitlines():
            if line.startswith('mtllib '):
                material = obj.parent / line.split(maxsplit=1)[1]
                assert material.is_file(), material
                for entry in material.read_text().splitlines():
                    if entry.strip().startswith('map_'):
                        texture = material.parent / entry.split()[-1]
                        assert texture.is_file(), texture


def test_rviz_marker_display_uses_transient_goal_topic():
    import yaml
    config = yaml.safe_load((ROOT / 'rviz/navigation.rviz').read_text())
    display = next(d for d in config['Visualization Manager']['Displays']
                   if d['Name'] == 'Assignment 5 Goals')
    assert display['Class'] == 'rviz_default_plugins/MarkerArray'
    assert display['Topic']['Value'] == '/assignment5/goal_markers'
    assert display['Topic']['Durability Policy'] == 'Transient Local'
