import os
import launch

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    gl_ros2_driver_udp_dir = os.path.join(get_package_share_directory('gl_ros2_driver_udp'), 'launch', 'gl_ros2_driver_udp.py')
    gl_ros2_driver_udp = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(gl_ros2_driver_udp_dir)
    )

    rviz_config_file = os.path.join(get_package_share_directory('gl_ros2_driver_udp'), 'rviz', 'config.rviz')
    rviz_node = Node(
        node_name = 'rviz2',
        package = 'rviz2',
        node_executable = 'rviz2',
        output = 'screen',
        arguments = ['-d', rviz_config_file],
    )

    ld = launch.LaunchDescription()
    ld.add_action( gl_ros2_driver_udp )
    ld.add_action( rviz_node )
    
    return ld
