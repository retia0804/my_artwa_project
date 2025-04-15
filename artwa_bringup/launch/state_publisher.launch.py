from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    로봇 상태를 발행하기 위한 launch 파일
    robot_state_publisher를 통해 로봇의 TF 정보를 발행합니다.
    """
    # 패키지 공유 디렉토리 경로 가져오기
    pkg_share = FindPackageShare("artwa_description").find("artwa_description")

    # launch 인자 선언
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # xacro를 통해 URDF 파일 로드
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "artwa_robot.urdf.xacro"]),
        ]
    )

    # 로봇 설명 파라미터 설정
    robot_description = {"robot_description": robot_description_content}

    # 노드 생성
    # 로봇 상태 발행 노드
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # launch description 생성 및 반환
    return LaunchDescription(
        [
            robot_state_publisher_node,
        ]
    )
