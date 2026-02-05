from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ns = LaunchConfiguration("ns")

    pkg_share = FindPackageShare("multistereo_planning_cloud")
    config_dir = PathJoinSubstitution([pkg_share, "config"])

    params_file = PathJoinSubstitution([config_dir, "params.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value="", description="ROS namespace"),

        Node(
            package="multistereo_planning_cloud",
            executable="multistereo_planning_cloud_node",
            namespace=ns,
            name="multistereo_planning_cloud_node",
            output="screen",
            parameters=[
                params_file,                    # load your YAML
                {"config_dir": config_dir},      # inject computed path
            ],
        )
    ])
