import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name='my_robot_chassis'

    bbot=IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'origintank.launch.py'    
                    )]), launch_arguments={'use_sim_time': 'true',
                                            'use_sim': 'true'}.items()
            )
    
    gazebo=IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'    
                    )])
            )

    spawn_entity=Node(
        package='gazebo_ros',executable='spawn_entity.py',
        arguments=['-topic','robot_description',
                    '-entity','bbot',
                    '-z','0.2'],
        output='screen'
    )

    joint_broad_spawner=Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state']
    )

    mecanum_spawner=Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller']
    )

    joint_wait_controll=RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner]
        )
    )

    mecanum_wait_controll=RegisterEventHandler(
        OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[mecanum_spawner]
        )
    )

    return LaunchDescription([
        bbot,
        gazebo,
        spawn_entity,
        joint_wait_controll,
        mecanum_wait_controll
    ])