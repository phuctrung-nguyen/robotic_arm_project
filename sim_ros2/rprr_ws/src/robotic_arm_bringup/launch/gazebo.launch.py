"""
Mô phỏng robot trong Gazebo (gz sim) kèm RViz.

Khởi chạy Gazebo, spawn robot từ URDF, bridge ROS <-> Gazebo và RViz.
Các khớp được điều khiển bằng plugin JointPositionController của Gazebo qua
topic ``/q1_cmd_pos`` ... ``/q4_cmd_pos``; ``/joint_states`` lấy từ Gazebo.
"""

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os

from ament_index_python.packages import get_package_share_path, get_package_share_directory


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_path('robotic_arm_description'),
        'urdf',
        'robotic_arm_gazebo.xacro'
    )

    gazebo_config_path = os.path.join(
        get_package_share_directory('robotic_arm_bringup'),
        'config',
        'gazebo_bridge.yaml'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('robotic_arm_description'),
        'rviz',
        'robotic_arm_config.rviz'
    )

    robot_description = ParameterValue(
        Command(['xacro', ' ', urdf_path]),
        value_type=str
    )

    gz_launch_path = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    # Cho Gazebo tìm được mesh dạng model://robotic_arm_description/meshes/...
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(get_package_share_directory('robotic_arm_description'))
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robotic_arm'
        ],
        output='screen'
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': gazebo_config_path
        }],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        gz_sim,
        robot_state_publisher_node,
        spawn_robot_node,
        bridge_node,
        rviz_node,
    ])
