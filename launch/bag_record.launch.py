from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction

def generate_bag_record_cmd(context, *args, **kwargs):
    bag_path = LaunchConfiguration('bag_path').perform(context)
    output_topic = LaunchConfiguration('output_topic')
    additional_topics_str = LaunchConfiguration('additional_topics').perform(context)
    
    # Split space-seperated additional topics string into a list
    additional_topics = additional_topics_str.split()
    
    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_path, output_topic] + additional_topics,
            output='screen'
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        # Declare parameters for recording compressed bag
        DeclareLaunchArgument(
            'bag_path',
            default_value='./my_bag',
            description='Path to save the compressed ROS 2 bag file'
        ),


        DeclareLaunchArgument(
            'output_topic',
            default_value='/decompressed_pointcloud_draco',
            description='Output decompressed point cloud topic'
        ),

        # add additional topics from original to include in output rosbag
        DeclareLaunchArgument(
            'additional_topics',
            default_value='',
            description='Space-seperated list of additional topics to record into output bag'
        ),

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

        # Dynamically generate ros2 bag record command
        OpaqueFunction(function=generate_bag_record_cmd),

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
