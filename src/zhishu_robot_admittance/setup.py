from glob import glob
import os

from setuptools import find_packages, setup


package_name = "zhishu_robot_admittance"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zhul",
    maintainer_email="zhu_liang@ota-os.com",
    description="Cooperative dual-arm Cartesian admittance control for box carrying.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "estimate_bottom_reaction = zhishu_robot_admittance.reaction_estimator:main",
            "dual_arm_admittance_controller = "
            "zhishu_robot_admittance.admittance_controller:main",
            "activate_admittance_controllers = "
            "zhishu_robot_admittance.controller_activation:main",
            "coordinate_admittance_grasp = "
            "zhishu_robot_admittance.grasp_coordinator:main",
            "run_admittance_validation = "
            "zhishu_robot_admittance.validation_runner:main",
        ],
    },
)
