from pathlib import Path

from setuptools import find_packages, setup


package_name = 'turtlebot4'
resource_directories = ('launch', 'config', 'maps', 'rviz', 'models', 'licenses')
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
]

# Preserve subdirectories so future map images and nested configs resolve correctly.
for directory in resource_directories:
    for path in sorted(Path(directory).rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts:
            data_files.append((str(Path('share') / package_name / path.parent),
                               [str(path)]))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test', 'tests']),
    data_files=data_files,
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='uonggiahuy',
    maintainer_email='uonggiahuy0411@gmail.com',
    description='TurtleBot 4 mapping, three-goal navigation and concurrent YOLO detection.',
    license='Apache-2.0',
    url='https://github.com/uonggiahuy/UR_HRI_assignments/tree/assignments_5',
    entry_points={'console_scripts': [
        'check_simulation = turtlebot4.diagnostics:main',
        'start_slam = turtlebot4.slam_runner:main',
        'teleop = turtlebot4.teleop:main',
        'save_map = turtlebot4.map_saver:main',
        'check_mapping = turtlebot4.mapping_diagnostics:main',
        'start_localization = turtlebot4.localization_runner:main',
        'set_initial_pose = turtlebot4.initial_pose:main',
        'check_localization = turtlebot4.localization_diagnostics:main',
        'start_navigation = turtlebot4.navigation_runner:main',
        'prepare_navigation = turtlebot4.prepare_navigation:main',
        'check_navigation = turtlebot4.navigation_diagnostics:main',
        'spawn_assignment5_models = turtlebot4.assignment5_scene:main',
        'waypoint_navigator = turtlebot4.waypoint_navigator:main',
        'goal_markers = turtlebot4.goal_markers:main',
        'yolo_viewer = turtlebot4.yolo_viewer:main',
        'yolo_detector = turtlebot4.yolo_detector:main',
    ]},
)
