from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments (optional overrides)
        DeclareLaunchArgument(
            'csv_folder_path',
            default_value='/tmp',
            description='Path to save CSV logs'
        ),
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/velodyne/velodyne_points',
            description='Input point cloud topic'
        ),
        DeclareLaunchArgument(
            'quantization_bits',
            default_value='14',
            description='Quantization bits for Draco compression'
        ),
        DeclareLaunchArgument(
            'encoding_speed',
            default_value='10',
            description='Draco encoding speed (0=slowest, 10=fastest)'
        ),
        DeclareLaunchArgument(
            'decoding_speed',
            default_value='10',
            description='Draco decoding speed (0=slowest, 10=fastest)'
        ),

        # Compressor Node
        Node(
            package='draco_ros2',
            executable='compress',
            name='compress',
            parameters=[{
                'csv_folder_path': LaunchConfiguration('csv_folder_path'),
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'quantization_bits': LaunchConfiguration('quantization_bits'),
                'encoding_speed': LaunchConfiguration('encoding_speed'),
                'decoding_speed': LaunchConfiguration('decoding_speed'),
            }],
            output='screen'
        ),

        # Decompressor Node
        Node(
            package='draco_ros2',
            executable='decompress',
            name='decompress',
            parameters=[{
                'csv_folder_path': LaunchConfiguration('csv_folder_path'),
            }],
            output='screen'
        ),
    ])
