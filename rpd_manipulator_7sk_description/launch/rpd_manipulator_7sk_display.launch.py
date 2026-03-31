from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from pathlib import Path
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    urdf_path = Path(get_package_share_path('rpd_manipulator_7sk_description')) / 'urdf' / 'rpd_manipulator_7sk.urdf.xacro'
    rviz_config_path = Path(get_package_share_path('rpd_manipulator_7sk_description')) / 'rviz' / 'rpd_manipulator_7sk.rviz'

    robot_description = ParameterValue(Command(['xacro', ' ', urdf_path]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
        output="screen"
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen"
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])