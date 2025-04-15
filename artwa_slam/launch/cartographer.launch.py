from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Cartographer SLAM을 실행하기 위한 launch 파일
    로봇의 센서 데이터를 기반으로 지도를 생성하고 로봇의 위치를 추정합니다.
    """
    # 패키지 공유 디렉토리 경로 가져오기
    slam_pkg_share = FindPackageShare("artwa_slam").find("artwa_slam")

    # LaunchConfiguration을 통해 인자 값 가져오기
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    configuration_basename = LaunchConfiguration(
        "configuration_basename", default="slam.lua"
    )

    # 노드 생성
    # Cartographer 노드
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory",
            PathJoinSubstitution([slam_pkg_share, "config"]),
            "-configuration_basename",
            configuration_basename,
        ],
    )

    # Cartographer 지도 생성 노드
    cartographer_occupancy_grid_node = Node(
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
            PathJoinSubstitution([slam_pkg_share, "rviz", "slam.rviz"]),
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # launch description 생성 및 반환
    return LaunchDescription(
        [
            # launch 인자 선언
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "configuration_basename",
                default_value=configuration_basename,
                description="Configuration file basename for Cartographer",
            ),
            # Nodes
            cartographer_node,
            cartographer_occupancy_grid_node,
            rviz_node,
        ]
    )
