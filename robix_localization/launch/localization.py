import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robix_localization_dir = get_package_share_directory("robix_localization")

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(robix_localization_dir,"config","ekf.yaml")]
)



    return LaunchDescription([
        #model_arg,
        #joint_state_publisher_gui_node,
        robot_localization_node
        #robot_state_publisher_node
        #rviz_node
    ])


