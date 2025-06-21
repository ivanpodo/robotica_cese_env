from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    robot_model = 'mycobot_280'

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('mycobot_gazebo'),
                'launch',
                'mycobot.gazebo.launch.py'
            )
        ]),
        launch_arguments={
            'load_controllers': 'true',
            'world_file': 'pick_and_place_demo.world',
            'use_camera': 'true',
            'use_rviz': 'false',
            'use_robot_state_pub': 'true',
            'use_sim_time': 'true',
            'robot_model': robot_model,
            'x': '0.0',
            'y': '0.0',
            'z': '0.05',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0'
        }.items()
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('mycobot_moveit_config'),
                'launch',
                'move_group.launch.py'
            )
        ]),
        launch_arguments={'robot_name': robot_model}.items()
    )

    hello_moveit_node = Node(
        package='tpf_ros2',
        executable='hello_moveit',
        name='hello_moveit',
        output='screen',
        parameters=[
            {'use_sim_time': True}])

    return LaunchDescription([
        SetEnvironmentVariable('ROBOT_MODEL', robot_model),
        gazebo_launch,
        moveit_launch,
        TimerAction(
            period=30.0,
            actions=[hello_moveit_node]
        )
    ])