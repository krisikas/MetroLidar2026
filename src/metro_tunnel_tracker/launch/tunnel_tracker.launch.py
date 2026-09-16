import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('metro_tunnel_tracker')
    default_params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/lidar_points',
        description='LiDAR PointCloud2 topic name'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to parameter file'
    )

    qos_reliability_arg = DeclareLaunchArgument(
        'qos_reliability',
        default_value='reliable',
        description='Subscription QoS reliability: reliable (for bag play) or best_effort'
    )

    tracker_node = Node(
        package='metro_tunnel_tracker',
        executable='tunnel_tracker_node',
        name='tunnel_tracker_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'lidar_topic': LaunchConfiguration('lidar_topic'),
                'qos_reliability': LaunchConfiguration('qos_reliability')
            }
        ]
    )

    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'send_buffer_limit': 100000000,
        }]
    )

    return LaunchDescription([
        lidar_topic_arg,
        params_file_arg,
        qos_reliability_arg,
        tracker_node,
        foxglove_node
    ])


