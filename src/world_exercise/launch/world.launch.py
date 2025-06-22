from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare('world_exercise'),
        'urdf',
        LaunchConfiguration('urdf_file')
    ])

    # Usar OpaqueFunction para leer el URDF en tiempo de ejecución
    from launch.actions import OpaqueFunction

    def load_robot_description(context):
        urdf_file = os.path.join(
            os.path.expanduser(
                context.perform_substitution(FindPackageShare('world_exercise'))
            ),
            'urdf',
            context.perform_substitution(LaunchConfiguration('urdf_file'))
        )
        with open(urdf_file, 'r') as infp:
            robot_description = infp.read()
        return [Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulated clock'
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value='arm_robot.urdf',
            description='URDF file to use'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='exercise_world.world',
            description='World file for Gazebo'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments=[(
                'gz_args',
                ['-r -v 1 ', PathJoinSubstitution([
                    FindPackageShare('world_exercise'),
                    'worlds',
                    LaunchConfiguration('world_name')
                ])]
            )]
        ),

        OpaqueFunction(function=load_robot_description),

        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-topic', '/robot_description',
                '-name', 'five_link_arm',
                '-x', '0.0', '-y', '0.0', '-z', '0.50',
                '--static'
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),
    ])