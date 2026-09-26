from setuptools import find_packages, setup


package_name = 'zhishu_robot_commander_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhul',
    maintainer_email='zhu_liang@ota-os.com',
    description='Customer-facing Python client for the dual-arm Commander.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander_example = zhishu_robot_commander_py.example:main',
            'torso_head_example = zhishu_robot_commander_py.torso_head_example:main',
        ],
    },
)
