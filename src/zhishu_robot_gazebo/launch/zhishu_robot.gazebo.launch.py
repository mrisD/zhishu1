#!/usr/bin/env python3
"""
Launch Gazebo simulation with a robot.

This launch file sets up a complete ROS 2 simulation environment with Gazebo for
a zhishu_robot robot.

:author: Zhul
:date: June 26, 2026
"""

import os
import sys
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PythonExpression


def generate_launch_description():
    """
    Generate a launch description for the Gazebo simulation.

    This function sets up all necessary parameters, paths, and nodes required to launch
    the Gazebo simulation with a robot. It handles:
    1. Setting up package paths and constants
    2. Declaring launch arguments for robot configuration
    3. Setting up the Gazebo environment
    4. Spawning the robot in simulation

    Returns:
        LaunchDescription: A complete launch description for the simulation
    """
    # Constants for paths to different files and folders
    package_name_gazebo = 'zhishu_robot_gazebo'
    package_name_description = 'zhishu_robot_description'
    package_name_moveit = 'zhishu_robot_moveit_config'

    default_robot_name = 'zhishu_robot'
    gazebo_models_path = 'models'
    default_world_file = 'pick_and_place_demo.world'
    gazebo_worlds_path = 'worlds'

    ros_gz_bridge_config_file_path = 'config/ros_gz_bridge.yaml'

    # Set the path to different files and folders
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    gz_ros2_control_lib = os.path.join(
        get_package_prefix('gz_ros2_control'), 'lib')
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    pkg_share_description = FindPackageShare(
        package=package_name_description).find(package_name_description)
    pkg_share_moveit = FindPackageShare(package=package_name_moveit).find(package_name_moveit)

    gazebo_models_path = os.path.join(pkg_share_gazebo, gazebo_models_path)
    default_ros_gz_bridge_config_file_path = os.path.join(
        pkg_share_gazebo, ros_gz_bridge_config_file_path)
    robot_xacro = os.path.join(
        pkg_share_moveit, 'config', 'zhishu_robot.urdf.xacro')
    controllers_yaml = os.path.join(
        pkg_share_moveit, 'config', 'ros2_controllers.yaml')
    initial_positions_file = LaunchConfiguration('initial_positions_file')
    use_head_camera = LaunchConfiguration('use_head_camera')
    hide_head_pitch_visual = LaunchConfiguration('hide_head_pitch_visual')
    handeye_target_enabled = LaunchConfiguration('handeye_target_enabled')
    bottle_grasp_sim_enabled = LaunchConfiguration('bottle_grasp_sim_enabled')
    mobile_base_sim_enabled = LaunchConfiguration('mobile_base_sim_enabled')
    managed_gazebo_shutdown = LaunchConfiguration('managed_gazebo_shutdown')
    tcp_calibration_enabled = LaunchConfiguration('tcp_calibration_enabled')
    tcp_calibration_tool = LaunchConfiguration('tcp_calibration_tool')
    tcp_calibration_arm = LaunchConfiguration('tcp_calibration_arm')
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', robot_xacro,
            ' use_gazebo:=true use_fake_hardware:=false initial_positions_file:=',
            initial_positions_file,
            ' use_head_camera:=', use_head_camera,
            ' hide_head_pitch_visual:=', hide_head_pitch_visual,
            ' handeye_target_enabled:=', handeye_target_enabled,
            ' bottle_grasp_sim_enabled:=', bottle_grasp_sim_enabled,
            ' mobile_base_sim_enabled:=', mobile_base_sim_enabled,
            ' mobile_base_sim_mass:=', LaunchConfiguration('mobile_base_sim_mass'),
            ' waist_pitch_min:=', LaunchConfiguration('waist_pitch_min'),
            ' tote_finger_length:=', LaunchConfiguration('tote_finger_length'),
            ' tcp_calibration_enabled:=', tcp_calibration_enabled,
            ' tcp_calibration_tool:=', tcp_calibration_tool,
            ' tcp_calibration_arm:=', tcp_calibration_arm,
        ]),
        value_type=str,
    )

    # Launch configuration variables
    load_controllers = LaunchConfiguration('load_controllers')
    use_left_arm_controller = LaunchConfiguration('use_left_arm_controller')
    headless = LaunchConfiguration('headless')
    paused = LaunchConfiguration('paused')
    force_x11 = LaunchConfiguration('force_x11')
    render_engine_gui = LaunchConfiguration('render_engine_gui')
    software_rendering = LaunchConfiguration('software_rendering')
    robot_name = LaunchConfiguration('robot_name')
    use_camera = LaunchConfiguration('use_camera')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')

    world_path = PathJoinSubstitution([
        pkg_share_gazebo,
        gazebo_worlds_path,
        world_file
    ])

    # Set the pose configuration variables
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    allow_renaming = LaunchConfiguration('allow_renaming')

    # Declare the launch arguments
    declare_tcp_calibration_enabled_cmd = DeclareLaunchArgument(
        'tcp_calibration_enabled', default_value='false',
        description='Simulation-only pivot probes, disabled by default')
    declare_tcp_calibration_tool_cmd = DeclareLaunchArgument(
        'tcp_calibration_tool', default_value='probe',
        description='Calibration tool ID: probe or long_probe')

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value=default_robot_name,
        description='The name for the robot')

    declare_load_controllers_cmd = DeclareLaunchArgument(
        name='load_controllers',
        default_value='true',
        description='Flag to enable loading of ROS 2 controllers')

    declare_use_left_arm_controller_cmd = DeclareLaunchArgument(
        name='use_left_arm_controller',
        default_value='false',
        description=(
            'Use left_arm_controller instead of dual_arm_controller for '
            'left-arm-only execution'))

    declare_headless_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='false',
        description='Run only the Gazebo server (no GUI)')

    declare_paused_cmd = DeclareLaunchArgument(
        name='paused',
        default_value='false',
        description='Start Gazebo paused so controllers can activate before gravity')

    declare_force_x11_cmd = DeclareLaunchArgument(
        name='force_x11',
        default_value='false',
        description='Force Qt to use X11; useful for Gazebo GUI in VMware/Wayland')

    declare_render_engine_gui_cmd = DeclareLaunchArgument(
        name='render_engine_gui',
        default_value='ogre2',
        description='Gazebo GUI rendering engine (ogre2 or compatibility engine ogre)')

    declare_software_rendering_cmd = DeclareLaunchArgument(
        name='software_rendering',
        default_value='false',
        description='Use Mesa software OpenGL; slower but stable in virtual machines')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='true',
        description='Flag to enable robot state publisher')

    # GUI and visualization arguments
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui')

    declare_use_camera_cmd = DeclareLaunchArgument(
        name='use_camera',
        default_value='false',
        description='Flag to enable the RGBD camera for Gazebo point cloud simulation')

    declare_use_head_camera_cmd = DeclareLaunchArgument(
        name='use_head_camera',
        default_value='false',
        description='Enable the head RGB camera used by hand-eye calibration')

    declare_hide_head_pitch_visual_cmd = DeclareLaunchArgument(
        name='hide_head_pitch_visual',
        default_value='false',
        description='Hide only the rendered head_pitch_link mesh')

    declare_handeye_target_enabled_cmd = DeclareLaunchArgument(
        name='handeye_target_enabled',
        default_value='false',
        description='Replace the rendered left gripper with the flange-mounted ChArUco target')

    declare_bottle_grasp_sim_enabled_cmd = DeclareLaunchArgument(
        name='bottle_grasp_sim_enabled',
        default_value='false',
        description='Enable the detachable joint used by bottle grasp simulation')

    declare_mobile_base_sim_enabled_cmd = DeclareLaunchArgument(
        name='mobile_base_sim_enabled',
        default_value='false',
        description='Enable the Gazebo DiffDrive plugin and /cmd_vel /odom /tf topics')

    declare_managed_gazebo_shutdown_cmd = DeclareLaunchArgument(
        name='managed_gazebo_shutdown',
        default_value='false',
        description='Work around known Gazebo teardown crashes')

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='true',
        description='Flag to enable Gazebo')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Flag to enable RViz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_world_cmd = DeclareLaunchArgument(
        name='world_file',
        default_value=default_world_file,
        description='World file name (e.g., empty.world, house.world, pick_and_place_demo.world)')

    declare_initial_positions_cmd = DeclareLaunchArgument(
        name='initial_positions_file',
        default_value=os.path.join(pkg_share_moveit, 'config', 'initial_positions.yaml'),
        description='Absolute path to the ros2_control initial joint positions YAML')

    # Pose arguments
    declare_x_cmd = DeclareLaunchArgument(
        name='x',
        default_value='0.0',
        description='x component of initial position, meters')

    declare_y_cmd = DeclareLaunchArgument(
        name='y',
        default_value='0.0',
        description='y component of initial position, meters')

    declare_z_cmd = DeclareLaunchArgument(
        name='z',
        default_value='0.05',
        description='z component of initial position, meters')

    declare_roll_cmd = DeclareLaunchArgument(
        name='roll',
        default_value='0.0',
        description='roll angle of initial orientation, radians')

    declare_pitch_cmd = DeclareLaunchArgument(
        name='pitch',
        default_value='0.0',
        description='pitch angle of initial orientation, radians')

    declare_yaw_cmd = DeclareLaunchArgument(
        name='yaw',
        default_value='0.0',
        description='yaw angle of initial orientation, radians')

    declare_allow_renaming_cmd = DeclareLaunchArgument(
        name='allow_renaming',
        default_value='true',
        description=(
            'Allow Gazebo to append a suffix when the model name already exists. '
            'Transfer runs disable this so /cmd_vel always controls the intended robot.'))

    # Publish the same Jazzy/Gazebo-ready URDF that is spawned below.
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(use_robot_state_pub)
    )

    # Gazebo's controller manager owns the simulated arm joints. The normal
    # simulation uses the combined controller; hand-eye pose sampling needs a
    # left-arm-only controller because MoveIt sends goals to left_arm.
    gazebo_controller_spawner_dual_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'body_posture_controller',
            'dual_arm_controller',
            'left_gripper_controller',
            'right_gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
            '--param-file', controllers_yaml,
        ],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", load_controllers, "' == 'true' and '",
            use_left_arm_controller, "' == 'false'"
        ])),
    )

    gazebo_controller_spawner_left_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'body_posture_controller',
            'left_arm_controller',
            'left_gripper_controller',
            'right_gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
            '--param-file', controllers_yaml,
        ],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", load_controllers, "' == 'true' and '",
            use_left_arm_controller, "' == 'true'"
        ])),
    )

    # Gazebo resolves model:// assets from this package's models directory.
    set_env_vars_model_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gazebo_models_path)
    # The robot uses package://zhishu_robot_description/meshes/... URIs. Gazebo
    # needs the parent share directory so it can resolve the package name and
    # render the visual meshes (entity creation can succeed even if visuals
    # cannot be resolved).
    set_env_vars_robot_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(pkg_share_description))
    # Gazebo Sim does not reliably discover ROS 2 system plugins from
    # LD_LIBRARY_PATH alone.  Make the gz_ros2_control plugin directory
    # available to every Gazebo launch, including the hand-eye simulation.
    set_env_vars_system_plugins = AppendEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        gz_ros2_control_lib)

    set_qt_x11 = SetEnvironmentVariable(
        'QT_QPA_PLATFORM',
        'xcb',
        condition=IfCondition(force_x11),
    )
    set_software_rendering = SetEnvironmentVariable(
        'LIBGL_ALWAYS_SOFTWARE',
        '1',
        condition=IfCondition(software_rendering),
    )
    # Headless Gazebo still needs a real X display for the OGRE v1 (GLX) render
    # engine used by the camera sensors.  ros_gz_sim's gz_sim.launch.py only
    # forwards GZ_SIM_* env vars, so DISPLAY/XAUTHORITY must be set explicitly
    # here or the gz sim child process loses the X connection and fails with
    # "Unable to create glx context".  Pin to the user's Xwayland display.
    set_display = SetEnvironmentVariable(
        'DISPLAY', os.environ.get('DISPLAY', ':0'))
    set_xauthority = SetEnvironmentVariable(
        'XAUTHORITY', os.environ.get('XAUTHORITY', ''))

    # Start Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [
            ' -v 4 ',
            PythonExpression(["'' if '", paused, "' == 'true' else '-r'"]),
            ' --render-engine-gui ', render_engine_gui,
            ' ', world_path,
        ])],
        condition=IfCondition(PythonExpression([
            "'", use_gazebo, "' == 'true' and '", headless, "' != 'true'",
        ])))

    start_gazebo_headless_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [
            ' -s --headless-rendering -v 4 ',
            PythonExpression(["'' if '", paused, "' == 'true' else '-r'"]),
            ' ', world_path,
        ])],
        condition=IfCondition(PythonExpression([
            "'", use_gazebo, "' == 'true' and '", headless,
            "' == 'true' and '", managed_gazebo_shutdown, "' != 'true'",
        ])))

    managed_gazebo_headless_cmd = ExecuteProcess(
        cmd=(['gdb', '-batch', '-ex', 'run', '-ex', 'info registers', '-ex', 'x/8i $pc-8', '-ex', 'thread apply all bt 12', '--args'] + [
            '/usr/bin/ruby', '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz', 'sim', '-s', '--headless-rendering', '-v', '4', '-r', world_path,
        ] if os.environ.get('PALLET_GZ_TRACE') == '1' else [
            sys.executable,
            os.path.join(
                pkg_share_gazebo,
                '..',
                '..',
                'lib',
                package_name_gazebo,
                'managed_process.py',
            ),
            '--managed-target',
            '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz',
            'sim',
            '-s',
            '--headless-rendering',
            '-v',
            '4',
            '-r',
            world_path,
            '--force-version',
            '8',
        ]),
        name='managed_gazebo',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", use_gazebo, "' == 'true' and '", headless,
            "' == 'true' and '", managed_gazebo_shutdown, "' == 'true'",
        ])),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': default_ros_gz_bridge_config_file_path,
        }],
        output='screen',
        condition=IfCondition(use_gazebo),
    )

    # Includes optimizations to minimize latency and bandwidth when streaming image data
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/camera_head/depth_image',
            '/camera_head/image',
        ],
        remappings=[
            ('/camera_head/depth_image', '/camera_head/depth/image_rect_raw'),
            ('/camera_head/image', '/camera_head/color/image_raw'),
        ],
        condition=IfCondition(use_camera),
    )

    # Spawn the robot
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-allow_renaming', allow_renaming,
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ],
        condition=IfCondition(PythonExpression(["'", use_gazebo, "' == 'true' and '", LaunchConfiguration('spawn_robot'), "' == 'true'"])))

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('spawn_robot', default_value='true'))

    # Declare the launch options
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_paused_cmd)
    ld.add_action(declare_force_x11_cmd)
    ld.add_action(declare_render_engine_gui_cmd)
    ld.add_action(declare_software_rendering_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_tcp_calibration_enabled_cmd)
    ld.add_action(declare_tcp_calibration_tool_cmd)
    ld.add_action(DeclareLaunchArgument('tcp_calibration_arm', default_value='both'))
    ld.add_action(declare_load_controllers_cmd)
    ld.add_action(declare_use_left_arm_controller_cmd)
    ld.add_action(declare_use_camera_cmd)
    ld.add_action(declare_use_head_camera_cmd)
    ld.add_action(declare_hide_head_pitch_visual_cmd)
    ld.add_action(declare_handeye_target_enabled_cmd)
    ld.add_action(declare_bottle_grasp_sim_enabled_cmd)
    ld.add_action(declare_mobile_base_sim_enabled_cmd)
    ld.add_action(DeclareLaunchArgument('mobile_base_sim_mass', default_value='11.9793854739169'))
    ld.add_action(DeclareLaunchArgument('waist_pitch_min', default_value='-1.57079632679'))
    ld.add_action(DeclareLaunchArgument('tote_finger_length', default_value='0.200'))
    ld.add_action(declare_managed_gazebo_shutdown_cmd)
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_initial_positions_cmd)

    # Add pose arguments
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_allow_renaming_cmd)

    # Add the actions to the launch description
    ld.add_action(set_env_vars_model_resources)
    ld.add_action(set_env_vars_robot_resources)
    ld.add_action(set_env_vars_system_plugins)
    ld.add_action(set_qt_x11)
    ld.add_action(set_software_rendering)
    ld.add_action(set_display)
    ld.add_action(set_xauthority)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_gazebo_headless_cmd)
    ld.add_action(managed_gazebo_headless_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_image_bridge_cmd)
    # The robot spawn (ros_gz_sim create) races gz sim's render-context init:
    # loading the robot meshes while the OGRE GLX render context is still being
    # created can make "Unable to create glx context" fire and segfault gz sim
    # on GPU-contended Xwayland.  Give gz sim a short settling window so the
    # render context is fully initialized before the robot meshes are inserted.
    ld.add_action(TimerAction(period=8.0, actions=[start_gazebo_ros_spawner_cmd]))
    # gz_ros_control creates /controller_manager only after the robot entity
    # has been inserted into Gazebo.  Starting the spawner in the same launch
    # tick races that initialization and can leave joint_state_broadcaster
    # absent, which makes MoveIt consume an invalid/stale joint state.  Give
    # Gazebo a short deterministic settling window before loading controllers.
    ld.add_action(TimerAction(period=20.0, actions=[gazebo_controller_spawner_dual_cmd]))
    ld.add_action(TimerAction(period=20.0, actions=[gazebo_controller_spawner_left_cmd]))

    return ld
