import os
import launch

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    gl_ros2_driver_udp = Node(
        node_name = 'gl_ros2_driver_udp',
        package = 'gl_ros2_driver_udp',
        node_executable = 'gl_ros2_driver_udp_node',
        output = 'screen',
        parameters = [
            {'gl_ip': '10.110.1.2'},
            {'gl_port': 2000},
            {'pc_port': 3000},
            {'frame_id': 'laser'},
            {'pub_topicname_lidar': 'scan'},
        ],
    )

    ld = launch.LaunchDescription()
    ld.add_action( gl_ros2_driver_udp )
    
    return ld
