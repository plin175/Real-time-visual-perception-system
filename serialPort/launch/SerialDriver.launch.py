# SerialDriver.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(get_package_share_directory('serialPort'), 'config', 'serialPort.yaml')

    serial_port_node = Node(
        package='serialPort',
        executable='serialPort_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([serial_port_node])
