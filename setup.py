from glob import glob
from os.path import join

from setuptools import find_packages, setup


PACKAGE_NAME = "ur3_llm_control"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
        (join("share", PACKAGE_NAME, "config"), glob("config/*.yaml")),
        (join("share", PACKAGE_NAME, "prompt"), glob("prompt/*.txt")),
        (join("share", PACKAGE_NAME, "srdf"), glob("srdf/*.xacro")),
        (join("share", PACKAGE_NAME, "urdf"), glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gia Huy",
    maintainer_email="maintainer@example.com",
    description="Assignment-local ROS 2 package for UR3 and UR3e skill-based planning.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "command_node = ur3_llm_control.command_node:main",
            "gripper_test = ur3_llm_control.gripper_test:main",
            "moveit_test = ur3_llm_control.moveit_test:main",
            "planning_scene = ur3_llm_control.planning_scene:main",
            "planning_scene_test = ur3_llm_control.planning_scene_test:main",
            "motion_primitives_test = ur3_llm_control.motion_primitives_test:main",
            "gazebo_attachment_sync = ur3_llm_control.gazebo_sync_node:main",
            "m65_phase = ur3_llm_control.m65_phases:main",
            "robot_skills_test = ur3_llm_control.robot_skills_test:main",
            "m8_validator_test = ur3_llm_control.m8_validator_test:main",
            "m8_plan_test = ur3_llm_control.m8_plan_test:main",
            "m8_transition_diagnostic = ur3_llm_control.m8_transition_diagnostic:main",
            "student_task_test = ur3_llm_control.student_task_test:main",
        ],
    },
)
