from glob import glob
from setuptools import find_packages, setup


package_name = "zhishu_robot_receiving_sim"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="zhul",
    maintainer_email="zhu_liang@ota-os.com",
    description="Gazebo receiving workflow simulation for the Zhishu robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "les_mock = zhishu_robot_receiving_sim.les_mock:main",
            "scanner_mock = zhishu_robot_receiving_sim.scanner_mock:main",
            "receiving_orchestrator = "
            "zhishu_robot_receiving_sim.receiving_orchestrator:main",
            "receiving_acceptance_test = "
            "zhishu_robot_receiving_sim.receiving_acceptance_test:main",
        ],
    },
)
