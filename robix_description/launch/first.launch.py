import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Define nodes

    joystick_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("robix_controller"),
                "launch",
                "joystick_teleop.launch.py"
            )
        ])
    )
    simple_serial_transmitter_node = Node(
        package='robix_firmware',
        executable='simple_serial_transmitter',
        name='simple_serial_transmitter'
    )

    enco_to_odom_to_tf_node = Node(
        package='robix_firmware',
        executable='enco_to_odom_to_tf',
        name='enco_to_odom_to_tf'
    )

    rp_lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("rplidar_ros2"),
                "launch",
                "rplidar.launch.py"
            )
        ])
    )

    localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("robix_localization"),
                "launch",
                "local_localization.launch.py"
            )
        ])
    )

    # Create launch description
    return LaunchDescription([
        joystick_node,
        simple_serial_transmitter_node,
        enco_to_odom_to_tf_node,
        rp_lidar_node,
        localization_node
    ])
