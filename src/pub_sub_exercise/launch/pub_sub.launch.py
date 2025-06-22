from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'frequency',
            default_value='5',
            description='Frecuencia de publicación del publisher (Hz)'
        ),
        DeclareLaunchArgument(
            'max_count',
            default_value='50',
            description='Cantidad máxima que publica el publisher'
        ),
        DeclareLaunchArgument(
            'reset_at',
            default_value='50',
            description='Valor en el que el subscriber resetea el contador'
        ),
        Node(
            package='pub_sub_exercise',
            executable='publisher',
            name='cpp_publisher',
            parameters=[{
                'frequency': LaunchConfiguration('frequency'),
                'max_count': LaunchConfiguration('max_count')
            }]
        ),
        Node(
            package='pub_sub_exercise',
            executable='subscriber',
            name='cpp_subscriber',
            parameters=[{
                'reset_at': LaunchConfiguration('reset_at')
            }]
        )
    ])