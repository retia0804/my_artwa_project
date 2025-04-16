import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_share = get_package_share_directory("artwa_navigation")

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    map_dir = LaunchConfiguration(
        "map",
        default="/home/retia/map/map.yaml",
    )

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            pkg_share,
            "param",
            "nav2_params.yaml",
        ),
    )

    use_rviz = LaunchConfiguration("use_rviz", default="false")

    nav2_launch_file_dir = os.path.join(pkg_share, "launch")

    rviz_config_dir = os.path.join(
        pkg_share,
        "rviz",
        "navigation.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=map_dir,
                description="Full path to map file to load",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=param_dir,
                description="Full path to param file to load",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="True",
                description="Whether to run SLAM",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Whether to launch RViz",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_launch_file_dir, "/bringup.launch.py"]
                ),
                launch_arguments={
                    "map": map_dir,
                    "use_sim_time": use_sim_time,
                    "params_file": param_dir,
                    "slam": LaunchConfiguration("slam"),
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_dir],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
                condition=IfCondition(use_rviz),
            ),
        ]
    )
