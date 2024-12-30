from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Nod pentru SLAM Toolbox
            Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[
                    {
                        "use_sim_time": False,
                        "slam_toolbox_config_file": "/home/user/robot_ws/src/robot_controller/resource/slam_config.yaml",
                    }
                ],
                output="screen",
            ),
            # Nod pentru RViz
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    "/home/user/robot_ws/src/robot_controller/resource/robot.rviz",
                ],
                output="screen",
            ),
            # Nod pentru citirea datelor LiDAR
            Node(
                package="robot_controller",
                executable="serial_reader",
                name="serial_reader",
                output="screen",
            ),
            # Transformare statică odom -> base_link
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_odom_to_base",
                arguments=["0", "0", "0", "0", "0", "0", "1", "odom", "base_link"],
                output="screen",
            ),
            # Transformare statică base_link -> lidar_frame
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_base_to_lidar",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "1",
                    "base_link",
                    "lidar_frame",
                ],
                output="screen",
            ),
        ]
    )
