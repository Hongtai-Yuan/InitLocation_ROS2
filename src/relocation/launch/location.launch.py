import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    map_dir = "/home/yuan/fishbot/src/fishbot_navigation2/maps"
    map_yaml_path = os.path.join(map_dir, 'fishbot_map.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': map_yaml_path},
                {'frame_id': 'map'},
                {'topic_name': 'map'}
            ]
        ),
        Node(
            package='relocation',
            executable='relocation',
            name='relocation_node',
            output='screen'
        )
    ])
