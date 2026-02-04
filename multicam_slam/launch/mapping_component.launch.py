from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare("multistereo_planning_cloud"),
        "config",
        "params.yaml"
    ])

    container = ComposableNodeContainer(
        name="multistereo_planning_cloud_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="multistereo_planning_cloud",
                plugin="multicam_depth::MultiStereoPlanningCloudNode",
                name="multistereo_planning_cloud_node",
                parameters=[params],
            )
        ],
        output="screen",
    )

    return LaunchDescription([container])
