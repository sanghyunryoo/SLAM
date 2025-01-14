from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
        
    pkg_path = os.path.join(get_package_share_directory("flamingo_description"))
    xacro_file = os.path.join(pkg_path, "urdf", "flamingo_rev02.urdf.xacro")
    robot_description = xacro.process_file(xacro_file)
    
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='ouster64.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    
    declare_odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom_lio', description='Odometry topic name'
    )
    declare_lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic', default_value='/cloud_registered', description='LiDAR topic name'
    )
        
    flamingo_description_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description.toxml()},
            {"use_sim_time": use_sim_time}
        ],
    )
    
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        parameters=[
            {'/root/slam_ws/src/FAST-LIO-SAM-SC-QN/third_party/FAST_LIO/config/ouster64.yaml'},
            {'use_sim_time': use_sim_time}],
        output='screen',
        # remappings=[('/Odometry', '/odom_lio')]  # base_link to base_footprint remapping,
    )    
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )    

    fast_lio_sam_node = Node(
        package='fast_lio_sam_sc_qn',
        executable='fast_lio_sam_sc_qn_node',
        name='fast_lio_sam_sc_qn_node',
        output='screen',
        parameters=[
            {'/root/slam_ws/src/FAST-LIO-SAM-SC-QN/fast_lio_sam_sc_qn/config/config.yaml'},
            {'use_sim_time': use_sim_time}
        ],
    )

    pointcloud_to_grid_node = Node(
        package='pointcloud_to_grid',
        executable='pointcloud_to_grid_node',
        output='screen',
        parameters=[
            {'cloud_in_topic': "/corrected_current_pcd"},
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
        
    return LaunchDescription(
        [
            flamingo_description_node,
            declare_use_sim_time_cmd,
            # declare_config_path_cmd,
            # decalre_config_file_cmd,
            # declare_odom_topic_arg,
            # declare_lidar_topic_arg,
            # declare_rviz_cmd,
            # declare_rviz_config_path_cmd,
            # rviz_node,
            fast_lio_node,  
            fast_lio_sam_node,
            pointcloud_to_grid_node
        ]
    )
