from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch


def generate_launch_description():

    moveit_config = MoveItConfigsBuilder(robot_name='rpd_manipulator_7sk').to_moveit_configs()
    rviz_config_file = Path(moveit_config.package_path) / 'config' / 'moveit.rviz'

    launch_arguments = [
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('publish_robot_description_semantic', default_value='true'),
        DeclareLaunchArgument('allow_trajectory_execution', default_value='true'),
        DeclareLaunchArgument('publish_monitored_planning_scene', default_value='true'),
        DeclareLaunchArgument('capabilities', default_value=moveit_config.move_group_capabilities['capabilities']),
        DeclareLaunchArgument('disable_capabilities', default_value=moveit_config.move_group_capabilities['disable_capabilities'])
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    publish_robot_description_semantic = LaunchConfiguration('publish_robot_description_semantic')
    allow_trajectory_execution = LaunchConfiguration('allow_trajectory_execution')
    publish_monitored_planning_scene = LaunchConfiguration('publish_monitored_planning_scene')
    capabilities = ParameterValue(LaunchConfiguration('capabilities'), value_type=str)
    disable_capabilities = ParameterValue(LaunchConfiguration('disable_capabilities'), value_type=str)


    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': use_sim,
                'publish_robot_description_semantic': publish_robot_description_semantic,
                'allow_trajectory_execution': allow_trajectory_execution,
                'capabilities': capabilities,
                'disable_capabilities': disable_capabilities,
                'publish_planning_scene': publish_monitored_planning_scene,
                'publish_geometry_updates': publish_monitored_planning_scene,
                'publish_state_updates': publish_monitored_planning_scene,
                'publish_transforms_updates': publish_monitored_planning_scene,
                'monitor_dynamics': False,
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(start_rviz),
        executable='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config_file)],
        parameters=[
            {'use_sim_time': use_sim},
            moveit_config.to_dict()
        ],
    )


    return LaunchDescription([*launch_arguments, move_group_node, rviz_node])