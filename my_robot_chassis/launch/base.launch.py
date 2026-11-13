from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    config=PathJoinSubstitution([
        FindPackageShare('my_robot_chassis'),
        'config',
        'base.yaml'
    ])
    
    return LaunchDescription([
        Node(
            package='my_robot_chassis',
            executable='origintank_node',
            name='origintank_node',
            parameters=[config]
        )
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            # 参数顺序：X Y Z Yaw Pitch Roll 父坐标系 子坐标系
            arguments=['0.11', '0.0', '0.5', '0.0', '0.0', '0.0', 'base_footprint', 'camera_link']
        )])