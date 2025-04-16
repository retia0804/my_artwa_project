from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare the save_path argument
    save_path = LaunchConfiguration("save_path")

    # Create directory command
    mkdir_cmd = ExecuteProcess(
        cmd=["mkdir", "-p", save_path],
        output="screen",
    )

    # Create the map saver command
    map_saver_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-f",
            PathJoinSubstitution([save_path, "map"]),
        ],
        output="screen",
    )

    # Create the write_state service call command
    write_state_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/write_state",
            "cartographer_ros_msgs/srv/WriteState",
            [
                "{filename: '",
                PathJoinSubstitution([save_path, "map.pbstream"]),
                "', include_unfinished_submaps: 'true'}",
            ],
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "save_path",
                default_value="./map",
                description="Path where the map and pbstream will be saved (without extension)",
            ),
            mkdir_cmd,
            map_saver_cmd,
            write_state_cmd,
        ]
    )
