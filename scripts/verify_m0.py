"""Validate M0 packaging and the installed upstream audit baseline."""

import hashlib
import argparse
import importlib
import json
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from catkin_pkg.package import parse_package


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path,
                        help='Optional external JSON of upstream file SHA-256 values')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    expected_prefix = root.parents[1] / 'install' / 'turtlebot4'
    assert parse_package(str(root / 'package.xml')).name == 'turtlebot4'
    assert Path(get_package_prefix('turtlebot4')) == expected_prefix
    share = Path(get_package_share_directory('turtlebot4'))
    resources = ['package.xml', 'README.md', 'LICENSE', 'launch/README.md',
                 'config/README.md', 'maps/README.md', 'rviz/README.md']
    for relative in resources:
        assert (share / relative).read_bytes() == (root / relative).read_bytes(), relative
    importlib.import_module('turtlebot4')
    baseline = json.loads(args.baseline.read_text()) if args.baseline else {}
    for filename, expected in baseline.items():
        assert hashlib.sha256(Path(filename).read_bytes()).hexdigest() == expected, filename
    for package in ['turtlebot4_ignition_bringup', 'turtlebot4_navigation',
                    'slam_toolbox', 'nav2_bringup']:
        assert get_package_prefix(package) == '/opt/ros/humble', package
    print(f'PASS: manifest, import, prefix, {len(resources)} installed assets, '
          f'four upstream prefixes, {len(baseline)} unchanged upstream files')


if __name__ == '__main__':
    main()
