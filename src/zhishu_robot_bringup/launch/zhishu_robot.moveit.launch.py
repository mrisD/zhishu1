#!/usr/bin/env python3
from datetime import datetime
import os
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def _default_log_root():
    """Return <workspace>/zhishu_robot_logs beside colcon's log dir."""
    package_share = Path(
        get_package_share_directory("zhishu_robot_bringup")
    ).resolve()
    for parent in package_share.parents:
        if parent.name == "install":
            return parent.parent / "zhishu_robot_logs"
    return Path.cwd() / "zhishu_robot_logs"


def _configure_native_log_directory(context):
    """Put every process-level ROS log in this bringup session directory."""
    configured_root = LaunchConfiguration("log_root").perform(context).strip()
    session_name = LaunchConfiguration("log_session").perform(context).strip()
    if session_name in {".", ".."} or not re.fullmatch(
        r"[A-Za-z0-9_.-]+", session_name
    ):
        raise ValueError(
            "log_session must contain only letters, digits, '.', '_' or '-'"
        )
    if configured_root:
        root = Path(configured_root).expanduser()
    else:
        root = Path(
            os.environ.get("ZHISHU_ROBOT_LOG_DIR", str(_default_log_root()))
        ).expanduser()

    native_directory = (
        root / datetime.now().strftime("%Y-%m-%d") / session_name / "native"
    )
    native_directory.mkdir(parents=True, exist_ok=True)
    return [
        SetLaunchConfiguration("log_root", str(root)),
        SetEnvironmentVariable("ROS_LOG_DIR", str(native_directory)),
    ]


def generate_launch_description():
    session_started_at = datetime.now().strftime("%H-%M-%S")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    enable_left_arm_hardware = LaunchConfiguration("enable_left_arm_hardware")
    enable_right_arm_hardware = LaunchConfiguration("enable_right_arm_hardware")
    enable_torso_head_hardware = LaunchConfiguration("enable_torso_head_hardware")
    both_arm_hardware_enabled = AndSubstitution(
        enable_left_arm_hardware,
        enable_right_arm_hardware,
    )
    torso_head_hardware_disabled = NotSubstitution(enable_torso_head_hardware)
    arm_only_mode = AndSubstitution(
        both_arm_hardware_enabled,
        torso_head_hardware_disabled,
    )
    no_arm_hardware = NotSubstitution(both_arm_hardware_enabled)
    no_trajectory_hardware = AndSubstitution(
        no_arm_hardware,
        torso_head_hardware_disabled,
    )
    enable_motors_on_activate = LaunchConfiguration("enable_motors_on_activate")
    torso_head_enable_motors_on_activate = LaunchConfiguration(
        "torso_head_enable_motors_on_activate"
    )
    pdo_diagnostics_auto_start = LaunchConfiguration("pdo_diagnostics_auto_start")
    enable_log_recorder = LaunchConfiguration("enable_log_recorder")
    enable_kernel_ethercat_log = LaunchConfiguration("enable_kernel_ethercat_log")
    enable_commander = LaunchConfiguration("enable_commander")
    log_root = LaunchConfiguration("log_root")
    log_session = LaunchConfiguration("log_session")
    log_max_file_size_mb = LaunchConfiguration("log_max_file_size_mb")
    log_min_severity = LaunchConfiguration("log_min_severity")
    use_rviz = LaunchConfiguration("use_rviz")
    handeye_target_enabled = LaunchConfiguration("handeye_target_enabled")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    # ==============================
    # 包路径查找
    # ==============================
    robot_bringup_share = FindPackageShare("zhishu_robot_bringup").find("zhishu_robot_bringup")
    robot_moveit_config_share = FindPackageShare("zhishu_robot_moveit_config").find("zhishu_robot_moveit_config")
    commander_share = get_package_share_directory("zhishu_robot_commander_cpp")

    # ==============================
    # 配置文件路径
    # ==============================
    controllers_yaml = os.path.join(robot_moveit_config_share, "config", "ros2_controllers.yaml")
    rviz_config = os.path.join(robot_moveit_config_share, "config", "moveit.rviz")
    commander_yaml = os.path.join(commander_share, "config", "commander.yaml")

    # ==============================
    # MoveIt 配置构建
    # ==============================
    moveit_config = (
        MoveItConfigsBuilder("zhishu_robot", package_name="zhishu_robot_moveit_config")
        .robot_description(
            file_path="config/zhishu_robot.urdf.xacro",
            mappings={
                "use_gazebo": "false",
                "use_fake_hardware": use_fake_hardware,
                "enable_left_arm_hardware": enable_left_arm_hardware,
                "enable_right_arm_hardware": enable_right_arm_hardware,
                "enable_torso_head_hardware": enable_torso_head_hardware,
                "enable_motors_on_activate": enable_motors_on_activate,
                "torso_head_enable_motors_on_activate": (
                    torso_head_enable_motors_on_activate
                ),
                "pdo_diagnostics_auto_start": pdo_diagnostics_auto_start,
                "handeye_target_enabled": handeye_target_enabled,
                "initial_positions_file": initial_positions_file,
            },
        )
        .robot_description_semantic(file_path="config/zhishu_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    moveit_all_params = moveit_config.to_dict()

    # ==============================
    # Central file logger
    # ==============================
    # File I/O stays outside controller_manager's 1 kHz EtherCAT process.
    log_recorder_node = Node(
        package="zhishu_robot_logging",
        executable="zhishu_robot_log_recorder",
        name="zhishu_robot_log_recorder",
        output="screen",
        parameters=[{
            "log_root": ParameterValue(log_root, value_type=str),
            "session_name": ParameterValue(log_session, value_type=str),
            "max_file_size_mb": ParameterValue(log_max_file_size_mb, value_type=int),
            "min_severity": ParameterValue(log_min_severity, value_type=int),
        }],
        condition=IfCondition(enable_log_recorder),
    )

    kernel_ethercat_logger = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([
                FindPackagePrefix("zhishu_robot_logging"),
                "lib",
                "zhishu_robot_logging",
                "kernel_ethercat_logger",
            ]),
            "--log-root", log_root,
            "--session-name", log_session,
        ],
        output="screen",
        condition=IfCondition(enable_kernel_ethercat_log),
    )

    # ==============================
    # 1. robot_state_publisher
    # ==============================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_all_params, {"use_sim_time": False}],
    )

    # ==============================
    # 2. ros2_control_node
    # ==============================
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        prefix=["taskset -c 6"],  #controller_manager 固定到 CPU6
        output="screen",
        parameters=[controllers_yaml, {"use_sim_time": False}],
        remappings=[("~/robot_description", "/robot_description")],
        additional_env={"LC_NUMERIC": "C"},
    )

    # ==============================
    # 3. 控制器加载器
    # ==============================
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    dual_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "dual_arm_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    torso_head_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "torso_head_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    left_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_gripper_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    right_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_gripper_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    # ==============================
    # 4. move_group
    # ==============================
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_all_params,
            {"use_sim_time": False},
            {"allow_trajectory_execution": True},
            {"moveit_manage_controllers": True},
            # Publish the MoveIt model descriptions so an RViz instance on a
            # different ROS 2 host can build the same RobotModel.  Without
            # these, robot_description(_semantic) only exist as move_group
            # parameters and remote MotionPlanning reports an empty SRDF.
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
        ],
    )

    # ==============================
    # 5. Commander SDK ROS bridge
    # ==============================
    # Reuse the exact robot_description/SRDF/MoveIt parameters from this bringup.
    # The bridge is passive at startup and sends no motion until a command arrives.
    commander_node = Node(
        package="zhishu_robot_commander_cpp",
        executable="commander_node",
        name="zhishu_robot_commander",
        output="screen",
        parameters=[
            moveit_all_params,
            commander_yaml,
            {"use_sim_time": False},
        ],
        condition=IfCondition(enable_commander),
    )

    # ==============================
    # 6. RViz2（修复崩溃 + 移除无效 wait 命令）
    # ==============================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_all_params, {"use_sim_time": False}],
        additional_env={
            "LC_NUMERIC": "C",
            "QT_QPA_PLATFORM": "xcb",
            "RVIZ_DISABLE_LIBINPUT": "1"  # 防止RViz崩溃
        },
        condition=IfCondition(use_rviz),
    )

    # ==============================
    # 启动顺序
    # ==============================
    return LaunchDescription([
        # Hardware mode and components
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use mock joints instead of EtherCAT hardware",
        ),
        DeclareLaunchArgument(
            "enable_left_arm_hardware",
            default_value="true",
            description="Instantiate LeftArmSystem",
        ),
        DeclareLaunchArgument(
            "enable_right_arm_hardware",
            default_value="true",
            description="Instantiate RightArmSystem",
        ),
        DeclareLaunchArgument(
            "enable_torso_head_hardware",
            default_value="true",
            description=(
                "Enable TorsoHeadSystem. When combined with arm hardware, "
                "EtherCAT master/slave ownership must be verified separately."
            ),
        ),

        # Motor activation safety
        DeclareLaunchArgument(
            "enable_motors_on_activate",
            default_value="true",
            description=(
                "Enable CiA402 operation for the arm hardware; false keeps "
                "EtherCAT in communication-only mode"
            ),
        ),
        DeclareLaunchArgument(
            "torso_head_enable_motors_on_activate",
            default_value="false",
            description=(
                "Enable torso/head drives during hardware activation. Keep false "
                "for initial feedback/direction checks and enable explicitly by "
                "service."
            ),
        ),

        # Control and visualization nodes
        DeclareLaunchArgument(
            "enable_commander",
            default_value="false",
            description="Start the Commander ROS bridge after move_group starts",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2",
        ),
        DeclareLaunchArgument(
            "handeye_target_enabled",
            default_value="false",
            description=(
                "Replace the left gripper geometry with the flange-mounted "
                "ChArUco calibration target; the right gripper is unchanged"
            ),
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=os.path.join(
                robot_moveit_config_share,
                "config",
                "initial_positions.yaml",
            ),
            description=(
                "YAML file containing ros2_control initial joint positions; "
                "the hand-eye launch overrides this with its calibration pose"
            ),
        ),

        # EtherCAT/PDO diagnostics
        DeclareLaunchArgument(
            "pdo_diagnostics_auto_start",
            default_value="false",
            description=(
                "Capture a separate 20 s EtherCAT startup window; keep false for "
                "service-triggered cycle-only CSVs"
            ),
        ),

        # Logging
        DeclareLaunchArgument(
            "enable_log_recorder",
            default_value="true",
            description="Record /rosout to daily rotating text files",
        ),
        DeclareLaunchArgument(
            "enable_kernel_ethercat_log",
            default_value="true",
            description="Record new filtered kernel EtherCAT messages",
        ),
        DeclareLaunchArgument(
            "log_root",
            default_value=os.environ.get(
                "ZHISHU_ROBOT_LOG_DIR", str(_default_log_root())
            ),
            description=(
                "Log root; defaults to <workspace>/zhishu_robot_logs "
                "beside build, install, and log"
            ),
        ),
        DeclareLaunchArgument(
            "log_session",
            default_value=session_started_at,
            description=(
                "Per-launch directory name; defaults to bringup start time "
                "HH-MM-SS"
            ),
        ),
        DeclareLaunchArgument(
            "log_max_file_size_mb",
            default_value="10",
            description="Rotate the central log after this many MiB",
        ),
        DeclareLaunchArgument(
            "log_min_severity",
            default_value="10",
            description=(
                "Minimum severity: DEBUG=10 INFO=20 WARN=30 ERROR=40 FATAL=50"
            ),
        ),
        OpaqueFunction(function=_configure_native_log_directory),
        log_recorder_node,
        kernel_ethercat_logger,
        robot_state_publisher,
        ros2_control_node,

        joint_state_broadcaster_spawner,

        # Arm-only path: JSB -> dual arm -> grippers -> move_group.
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[dual_arm_controller_spawner],
            ),
            condition=IfCondition(arm_only_mode),
        ),

        # Whenever torso/head hardware is enabled, start its controller first.
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[torso_head_controller_spawner],
            ),
            condition=IfCondition(enable_torso_head_hardware),
        ),

        # Combined path: torso/head does not suppress the dual-arm chain.
        RegisterEventHandler(
            OnProcessExit(
                target_action=torso_head_controller_spawner,
                on_exit=[dual_arm_controller_spawner],
            ),
            condition=IfCondition(both_arm_hardware_enabled),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=dual_arm_controller_spawner,
                on_exit=[left_gripper_controller_spawner],
            ),
            condition=IfCondition(both_arm_hardware_enabled),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=left_gripper_controller_spawner,
                on_exit=[right_gripper_controller_spawner],
            ),
            condition=IfCondition(both_arm_hardware_enabled),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=right_gripper_controller_spawner,
                on_exit=[move_group_node],
            ),
            condition=IfCondition(both_arm_hardware_enabled),
        ),

        # Torso/head-only path ends directly at move_group.
        RegisterEventHandler(
            OnProcessExit(
                target_action=torso_head_controller_spawner,
                on_exit=[move_group_node],
            ),
            condition=UnlessCondition(both_arm_hardware_enabled),
        ),

        # Keep MoveIt available when no trajectory hardware is selected.
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[move_group_node],
            ),
            condition=IfCondition(no_trajectory_hardware),
        ),

        RegisterEventHandler(
            OnProcessStart(target_action=move_group_node, on_start=[commander_node])
        ),

        # Start RViz only after the selected controller chain reaches move_group.
        RegisterEventHandler(
            OnProcessStart(
                target_action=move_group_node,
                on_start=[rviz_node],
            )
        ),
    ])
