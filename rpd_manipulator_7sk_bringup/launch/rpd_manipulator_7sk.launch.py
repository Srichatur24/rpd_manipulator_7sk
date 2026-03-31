from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit, OnProcessStart
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    desc_pkg = get_package_share_directory('rpd_manipulator_7sk_description')
    bringup_pkg = get_package_share_directory('rpd_manipulator_7sk_bringup')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    launch_arguments = [
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='1000000'),
    ]
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    use_sim = LaunchConfiguration('use_sim')
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')

    xacro_file = Path(desc_pkg) / 'urdf' / 'rpd_manipulator_7sk.urdf.xacro'
    robot_controllers = Path(bringup_pkg) / 'config' / 'rpd_manipulator_7sk_controllers.yaml'
    gazebo_bridge_config_file = Path(bringup_pkg) / 'config' / 'gazebo_bridge_config.yaml'

    robot_description = ParameterValue(Command([
        'xacro', ' ', xacro_file, ' ',
        'use_sim:=', use_sim, ' ',
        'use_mock_hardware:=', use_mock_hardware, ' ',
        'port:=', port, ' ',
        'baudrate:=', baudrate
    ]), value_type=str)


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
        output="screen"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([Path(ros_gz_sim_pkg) / 'launch' / 'gz_sim.launch.py']),
        launch_arguments=[('gz_args', ['empty.sdf', ' -r'])],
        condition=IfCondition(use_sim),
    )

    gazebo_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description'],
        condition=IfCondition(use_sim),
        output='screen'
    )

    gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': str(gazebo_bridge_config_file)}],
        condition=IfCondition(use_sim),
        output='screen'
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        condition=UnlessCondition(use_sim),
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_state_broadcaster'],
        output="screen"
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['arm_controller'],
        output="screen"
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['gripper_controller'],
        output="screen"
    )


    return LaunchDescription([
        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo_spawn_node,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=control_node,
                on_start=[joint_state_broadcaster_spawner]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner, gripper_controller_spawner],
            )
        ),
        *launch_arguments,
        robot_state_publisher_node,
        gazebo_bridge_node,
        gazebo,
        gazebo_spawn_node,
        control_node
    ])