from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    """
    ARTWA 로봇의 기본 시스템을 실행하기 위한 launch 파일
    로봇 상태 발행, 라이다, RViz를 함께 실행합니다.
    """
    # 패키지 공유 디렉토리 경로 가져오기
    artwa_bringup_pkg_share = FindPackageShare("artwa_bringup").find("artwa_bringup")
    artwa_description_pkg_share = FindPackageShare("artwa_description").find(
        "artwa_description"
    )
    sllidar_pkg_share = FindPackageShare("sllidar_ros2").find("sllidar_ros2")

    # launch 인자 선언
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    use_rviz = LaunchConfiguration("use_rviz", default="false")

    # state_publisher.launch.py 실행
    state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [artwa_bringup_pkg_share, "launch", "state_publisher.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # sllidar sllidar_a2m8_launch.py 실행
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [sllidar_pkg_share, "launch", "sllidar_a2m8_launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # joint_state_publisher 노드
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # RViz2 시각화 노드
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([artwa_description_pkg_share, "rviz", "urdf.rviz"]),
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),

    # launch description 생성 및 반환
    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            # Launches
            state_publisher_launch,
            sllidar_launch,
            joint_state_publisher_node,
            rviz_node,
        ]
    )
