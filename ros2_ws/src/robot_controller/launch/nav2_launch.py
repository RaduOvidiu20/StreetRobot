from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Nod pentru lansarea Nav2
            Node(
                package="nav2_bringup",
                executable="bringup_launch.py",
                name="nav2_bringup",
                output="screen",
                parameters=[
                    {"use_sim_time": False},  # Dezactivează timpul de simulare
                    {"autostart": True},  # Pornește automat stack-ul Nav2
                    {"map_subscribe_transient_local": True},
                ],
            ),
            # Transformare statică map -> odom (dacă nu este setată de un alt nod)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_map_to_odom",
                arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom"],
                output="screen",
            ),
        ]
    )
