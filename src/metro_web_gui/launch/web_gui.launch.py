import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    web_gui_share = get_package_share_directory('metro_web_gui')
    web_dir_default = os.path.join(web_gui_share, 'web')

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='Port for HTTP/SSE Metro Web GUI server'
    )

    web_dir_arg = DeclareLaunchArgument(
        'web_dir',
        default_value=web_dir_default,
        description='Path to static web assets directory'
    )

    web_gui_node = Node(
        package='metro_web_gui',
        executable='web_gui_node',
        name='metro_web_gui_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'web_dir': LaunchConfiguration('web_dir'),
        }]
    )

    return LaunchDescription([
        port_arg,
        web_dir_arg,
        web_gui_node
    ])
