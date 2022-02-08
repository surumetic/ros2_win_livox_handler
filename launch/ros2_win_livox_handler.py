from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


config_file_dirname = 'rviz_config'
config_file_fname = 'livox_view.rviz'

rviz_config = os.path.join(
    get_package_share_directory('ros2_win_livox_handler'),
    config_file_dirname,
    config_file_fname)
print(rviz_config)

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_win_livox_handler',
            namespace='ros2_win_livox_handler',
            executable='ros2_win_livox_handler',
            output='screen'),
        Node(
            package='rviz2',
            namespace='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config]),            
    ])