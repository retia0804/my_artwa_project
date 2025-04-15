from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    ARTWA 로봇의 전체 시스템을 실행하기 위한 launch 파일
    로봇 상태 발행, SLAM, 라이다를 함께 실행합니다.
    """
    # 패키지 공유 디렉토리 경로 가져오기
    artwa_bringup_pkg_share = FindPackageShare("artwa_bringup").find("artwa_bringup")
    artwa_slam_pkg_share = FindPackageShare("artwa_slam").find("artwa_slam")
    sllidar_pkg_share = FindPackageShare("sllidar_ros2").find("sllidar_ros2")

    # launch 인자 선언
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    configuration_basename = LaunchConfiguration(
        "configuration_basename", default="artwa_2d.lua"
    )
    load_state_filename = LaunchConfiguration("load_state_filename", default="")
    save_state_filename = LaunchConfiguration("save_state_filename", default="")

    # state_publisher.launch.py 실행
    state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [artwa_bringup_pkg_share, "launch", "state_publisher.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # cartographer.launch.py 실행
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [artwa_slam_pkg_share, "launch", "cartographer.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "configuration_basename": configuration_basename,
            "load_state_filename": load_state_filename,
            "save_state_filename": save_state_filename,
        }.items(),
    )

    # sllidar a2m8_launch.py 실행
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sllidar_pkg_share, "launch", "a2m8_launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
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
                "configuration_basename",
                default_value="artwa_2d.lua",
                description="Configuration file basename for Cartographer",
            ),
            DeclareLaunchArgument(
                "load_state_filename",
                default_value="",
                description="Load state from file if not empty",
            ),
            DeclareLaunchArgument(
                "save_state_filename",
                default_value="",
                description="Save state to file if not empty",
            ),
            # Launches
            state_publisher_launch,
            cartographer_launch,
            sllidar_launch,
        ]
    )
