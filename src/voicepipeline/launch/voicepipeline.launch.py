from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    transcriber_node = Node(
        package='voicepipeline',
        executable='transcriber',
        name='transcriber',
        output='screen'
    )
    
    commander_node = Node(
        package='voicepipeline',
        executable='commander',
        name='commander',
        output='screen'
    )
    
    return LaunchDescription([
        transcriber_node,
        commander_node
    ])
