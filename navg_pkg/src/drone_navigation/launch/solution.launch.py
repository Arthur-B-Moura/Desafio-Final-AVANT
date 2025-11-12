from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # pkg_gancho_teste = get_package_share_directory("gancho_pkg_tester")
    # pkg_drone_vision = get_package_share_directory("drone_vision")
    # pkg_drone_navigation = get_package_share_directory("drone_navigation")


    # Include launch file to vision algorithm
    vision_alg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("drone_vision"),
                        "launch",
                        "vision.launch.py",
                    ]
                ),

            ]
        )
    )

    gancho_node = Node(
        package='gancho_pkg_tester',
        executable='gancho_node',
        name='gancho_node'
    )


    navigation_node = Node(
        package='drone_navigation',
        executable='navigator',
        name='navigator'
    )


    return LaunchDescription([
        gancho_node,
        navigation_node,
        vision_alg
    ])