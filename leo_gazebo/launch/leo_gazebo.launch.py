from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("leo_gazebo")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="world",
                default_value="",
                description="Path to the world file for gazebo",
            ),
            DeclareLaunchArgument(
                name="paused",
                default_value="false",
                description='Set to "true" to start gazebo server in a paused state.',
            ),
            DeclareLaunchArgument(
                name="gui",
                default_value="true",
                description='Set to "false" to run Gazebo headless.',
            ),
            DeclareLaunchArgument(name="debug", default_value="false"),
            DeclareLaunchArgument(
                name="verbose",
                default_value="false",
                description='Set to "true" to increase messages written to terminal.',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        gazebo_ros_share,
                        "/launch/gazebo.launch.py",
                    ]
                ),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "pause": LaunchConfiguration("paused"),
                    "gui": LaunchConfiguration("gui"),
                    "gdb": LaunchConfiguration("debug"),
                    "verbose": LaunchConfiguration("verbose"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [pkg_share, "/launch/spawn_robot.launch.py"]
                )
            ),
        ]
    )
