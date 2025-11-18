from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('pinky_tmproject')
    yaml_path = os.path.join(pkg_share, 'config', 'points.yaml')

    node = Node(
        package='pinky_tmproject',
        executable='tm_centerline_nav',
        name='tm_centerline_nav',
        output='screen',
        parameters=[{'points_yaml': yaml_path}]
    )

    return LaunchDescription([node])
