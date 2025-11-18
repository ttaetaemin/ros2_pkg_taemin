# ~/pinky/src/pinky_tmpatrol/launch/wallpatrol.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource  # ✅ Jazzy: XML은 launch_xml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml     = LaunchConfiguration('map')
    nav2_params  = LaunchConfiguration('params_file')

    # bringup XML (사용 중인 패키지)
    nav2_xml = PathJoinSubstitution([
        FindPackageShare('pinky_navigation'), 'launch', 'bringup_launch.xml'
    ])

    # 이 패키지 share (참고용)
    this_share = get_package_share_directory('pinky_tmpatrol')

    return LaunchDescription([
        # ----- 인자 -----
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map',          default_value='my_map.yaml'),
        # ✅ 기본 파라미터: pinky_navigation의 nav2_params.yaml 사용
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('pinky_navigation'), 'params', 'nav2_params.yaml'
            ])
        ),

        # ----- Map Server + AMCL -----
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_yaml}
            ]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # ----- bringup XML 포함 -----
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(nav2_xml),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml,
                'params_file': nav2_params
            }.items()
        ),

        # ----- 순찰 노드 -----
        Node(
            package='pinky_tmpatrol',
            executable='patrol_node',
            name='patrol_node',
            output='screen',
            parameters=[{'waypoints_file': 'waypoints.yaml'}]
        ),
    ])
