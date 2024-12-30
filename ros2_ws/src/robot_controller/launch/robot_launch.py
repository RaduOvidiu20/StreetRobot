import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obține calea completă către fișierul URDF
    urdf_file = os.path.join(get_package_share_directory('robot_controller'), 'resource', 'robot.urdf')

    # Citește conținutul fișierului URDF
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # Nodul pentru `static_transform_publisher` (pentru `lidar_frame`)
    static_transform_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "lidar_frame"],
        output="screen",
    )

    # Nodul pentru `static_transform_publisher` (pentru `wheel_link`)
    static_transform_wheel = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "wheel_link"],
        output="screen",
    )

    # Nodul pentru `robot_state_publisher`
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    # Nodul pentru RViz
    rviz_config_file = os.path.join(get_package_share_directory('robot_controller'), 'resource', 'robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Nodul pentru serial_reader
    serial_reader_node = Node(
        package='robot_controller',
        executable='serial_reader',
        name='serial_reader',
        output='screen',
    )

    # # Nodul pentru motor_controller
    # motor_controller_node = Node(
    #     package='robot_controller',
    #     executable='motor_controller',
    #     name='motor_controller',
    #     output='screen',
    # )

    # Returnăm descrierea lansării care include toate nodurile
    return LaunchDescription([
        static_transform_lidar,
        static_transform_wheel,
        robot_state_publisher_node,
        serial_reader_node,
        rviz_node,
        
    ])
