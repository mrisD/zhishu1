from glob import glob
from setuptools import find_packages, setup


package_name = "zhishu_robot_bottle_grasp"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/docs", glob("docs/*.md")),
        ("share/" + package_name + "/models", glob("models/*.pt")),
        ("share/" + package_name + "/scripts", glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zhul",
    maintainer_email="zhul@todo.todo",
    description="YOLO segmentation and RGB-D geometry for moving bottle grasping",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bottle_perception = zhishu_robot_bottle_grasp.bottle_perception_node:main",
            "moving_bottle = zhishu_robot_bottle_grasp.moving_bottle_node:main",
            "obstacle_cloud = zhishu_robot_bottle_grasp.obstacle_cloud_node:main",
            "planning_scene_obstacles = zhishu_robot_bottle_grasp.planning_scene_obstacles_node:main",
            "capture_yolo_dataset = zhishu_robot_bottle_grasp.dataset_capture_node:main",
        ],
    },
)
