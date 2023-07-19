from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    offboard_pub_node = Node(
        package='Final_NR_Wardi_Tracker_Stack',
        executable='offboard_pub',
        output='screen',
    )

    tracker_computation_node = Node(
        package='Final_NR_Wardi_Tracker_Stack',
        executable='nr_tracker_final.py',
        output='screen',
    )


    ld = LaunchDescription()
    ld.add_action(offboard_pub_node)
    ld.add_action(tracker_computation_node)
    # ld.add_action()
    return ld