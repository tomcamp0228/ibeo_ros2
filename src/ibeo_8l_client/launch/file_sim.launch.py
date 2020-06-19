import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    para_dir = os.path.join(get_package_share_directory('ibeo_8l_client'), 'config', 'para_file_sim.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('ibeo_8l_client'), 'config', 'default.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable("USER"), '_'],
            description='Prefix for node names'
        ),
        Node(
            package='ibeo_8l_client',
            node_namespace='ibeo_8l_client',
            node_executable='ibeo_file_sim',
            # node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'ibeo_file_sim'],
            node_name='ibeo_file_sim',
            parameters=[para_dir],
            remappings=None,
            arguments=None,
            output='screen',
        ),
        Node(
            package='ibeo_8l_client',
            node_namespace='ibeo_8l_client',
            node_executable='obj_visualization',
            # node_name=[LaunchConfiguration("node_prefix"), 'obj_visualization'],
            node_name='obj_visualization',
            parameters=[para_dir],
            # parameters=None,
            remappings=None,
            arguments=None,
            output='screen',
        ),
        Node(
            package='rviz2',
            node_namespace='rviz2',
            node_executable='rviz2',
            node_name=[LaunchConfiguration("node_prefix"), 'rviz2'],
            parameters=None,
            remappings=None,
            arguments=['-d',rviz_config_dir],
            output='screen',
        )
    ])