from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_dummy_cam',
            default_value='true',
            description='Use dummy camera instead of real usb_cam'
        ),
        DeclareLaunchArgument(
            'video_path',
            default_value='',
            description='Path to video file for dummy camera'
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/image_raw',
            description='Input image topic'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/converted_image',
            description='Output image topic'
        ),
        
        # Dummy USB camera node (conditionally launched)
        Node(
            package='image_conversion_pkg',
            executable='dummy_usb_cam_node',
            name='dummy_usb_cam',
            condition=launch.conditions.IfCondition(LaunchConfiguration('use_dummy_cam')),
            parameters=[{
                'video_path': LaunchConfiguration('video_path'),
                'output_topic': LaunchConfiguration('input_topic')
            }],
            output='screen'
        ),
        
        # Image conversion node
        Node(
            package='image_conversion_pkg',
            executable='image_conversion_node',
            name='image_conversion',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic')
            }],
            output='screen'
        )
    ])
