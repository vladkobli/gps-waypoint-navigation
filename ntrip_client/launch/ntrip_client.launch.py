import os
# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    param = os.path.join(
        get_package_share_directory('ntrip_client'),
        'config',
        'ntrip_client.param.yaml'
    )

    node=Node(
        package = 'ntrip_client',
        name = 'ntrip_client',
        executable = 'ntrip_client_exe',
        parameters = [param]
    )

    ld.add_action(node)
    return ld