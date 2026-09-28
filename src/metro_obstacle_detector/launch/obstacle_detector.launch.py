import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('metro_obstacle_detector')

    default_params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'params.yaml'
    ])

    # Declare arguments
    declare_lidar_topic = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/lidar_points',
        description='LiDAR PointCloud2 topic name'
    )

    declare_target_frame = DeclareLaunchArgument(
        'target_frame',
        default_value='hesai_lidar',
        description='Base sensor frame ID'
    )

    declare_qos = DeclareLaunchArgument(
        'qos_reliability',
        default_value='sensor_data',
        description='QoS reliability profile for cloud input: sensor_data, best_effort, or reliable'
    )

    declare_use_gpu = DeclareLaunchArgument(
        'use_gpu',
        default_value='false',
        description='Enable CUDA GPU acceleration'
    )

    declare_train_speed = DeclareLaunchArgument(
        'train_speed_mps',
        default_value='15.0',
        description='Train initial speed in m/s'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to parameter YAML file'
    )

    declare_web_gui = DeclareLaunchArgument(
        'launch_web_gui',
        default_value='true',
        description='Launch Metro Web GUI dashboard on port 8080'
    )

    declare_foxglove = DeclareLaunchArgument(
        'launch_foxglove',
        default_value='true',
        description='Launch Foxglove Studio WebSocket bridge on port 8765'
    )

    declare_bag_path = DeclareLaunchArgument(
        'bag_path',
        default_value='',
        description='Optional rosbag2 path to replay concurrently inside the container'
    )

    # Obstacle detector node
    detector_node = Node(
        package='metro_obstacle_detector',
        executable='metro_obstacle_detector_node',
        name='metro_obstacle_detector_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'lidar_topic': LaunchConfiguration('lidar_topic'),
                'target_frame': LaunchConfiguration('target_frame'),
                'qos_reliability': LaunchConfiguration('qos_reliability'),
                'use_gpu': LaunchConfiguration('use_gpu'),
                'train_speed_mps': LaunchConfiguration('train_speed_mps')
            }
        ]
    )

    # Web GUI node
    web_gui_node = Node(
        package='metro_web_gui',
        executable='web_gui_node',
        name='metro_web_gui_node',
        output='screen',
        parameters=[
            {
                'lidar_topic': LaunchConfiguration('lidar_topic')
            }
        ],
        condition=IfCondition(LaunchConfiguration('launch_web_gui'))
    )

    # Foxglove WebSocket Bridge
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': 8765}],
        condition=IfCondition(LaunchConfiguration('launch_foxglove'))
    )

    # In-container background rosbag player (if bag_path is specified)
    bag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path'), '--loop'],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('bag_path'), "' != ''"])
        )
    )

    return LaunchDescription([
        declare_lidar_topic,
        declare_target_frame,
        declare_qos,
        declare_use_gpu,
        declare_train_speed,
        declare_params_file,
        declare_web_gui,
        declare_foxglove,
        declare_bag_path,
        detector_node,
        web_gui_node,
        foxglove_node,
        bag_player
    ])
