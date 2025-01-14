from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument("topic", description="a pointcloud topic to process", default_value="nonground"),
        Node(
        package='pointcloud_to_grid',
        executable='pointcloud_to_grid_node',
        output='screen',
        parameters=[
            {'cloud_in_topic': "/cloud_registered"},
            {'position_x': 0.0},
            {'position_y': 0.0},
            {'intensity_factor': 10.0},
            {'height_factor': 10.0},
            {'verbose1': False},
            {'verbose2': False},
            {'cell_size': 0.05},
            {'length_x': 20.0},
            {'length_y': 30.0},
            #{'frame_out': 'os1_sensor'},
            {'mapi_topic_name': 'intensity_grid'},
            {'maph_topic_name': 'height_grid'},
            {'use_sim_time': True},
            {'filter_min_z': -2.0},
            {'filter_max_z': 5.0},
            {'occupied_threshold': 0.5},
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('/height_grid', '/map')]
        )

    ])

