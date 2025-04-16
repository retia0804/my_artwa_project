from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    """
    ARTWA 로봇의 localization 시스템을 실행하기 위한 launch 파일
    Cartographer를 사용하여 pure localization을 수행합니다.
    """
    # 패키지 공유 디렉토리 경로 가져오기
    artwa_slam_pkg_share = FindPackageShare("artwa_slam").find("artwa_slam")

    # launch 인자 선언
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    map_filename = LaunchConfiguration("map_filename")
    configuration_basename = LaunchConfiguration(
        "configuration_basename", default="localization.lua"
    )
    use_rviz = LaunchConfiguration("use_rviz", default="true")

    # Cartographer pure localization 실행
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory",
            PathJoinSubstitution([artwa_slam_pkg_share, "config"]),
            "-configuration_basename",
            configuration_basename,
            "-load_state_filename",
            map_filename,
        ],
    )

    # Cartographer occupancy grid 노드
    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-resolution", "0.05"],
    )

    # RViz2 시각화 노드
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([artwa_slam_pkg_share, "rviz", "slam.rviz"]),
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # launch description 생성 및 반환
    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "map_filename",
                default_value="/home/rail/test_map/map.pbstream",
                description="Full path to map pbstream file to load",
            ),
            DeclareLaunchArgument(
                "configuration_basename",
                default_value="localization.lua",
                description="Configuration file basename for Cartographer",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Whether to launch RViz",
            ),
            # Nodes
            cartographer_node,
            occupancy_grid_node,
            rviz_node,
        ]
    )
