import ament_index_python
import os
import launch
import launch_ros
import yaml

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    # #ros2 launch depthai_ros_driver camera.launch.py params_file:=/home/paul/Downloads/camera_all_sensors.yaml
    # oak_camera_params = os.path.join(ament_index_python.get_package_share_directory('vinerobot_bringup'),'config','camera_all_sensors.yaml')
    # oak_camera = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         ament_index_python.get_package_share_directory('depthai_ros_driver'), 'launch'), 
    #         '/camera.launch.py']),
    #     launch_arguments={'params_file': oak_camera_params}.items(),
    # )

    # joy_params = os.path.join(ament_index_python.get_package_share_directory('vinerobot_bringup'),'config','joystick.yaml')
    # joy_node = Node(
    #     package='joy',
    #     executable='joy_node',
    #     parameters=[joy_params],
    # )
    
    # teleop_node = Node(
    #     package='teleop_twist_joy', 
    #     executable='teleop_node',
    #     name = 'teleop_node',
    #     parameters=[joy_params],
    #     remappings=[('/cmd_vel', '/turtlesim1/turtle1/cmd_vel')]
    # )    

    # turtlesim_node = Node(
    #     package='turtlesim',
    #     namespace='turtlesim1',
    #     executable='turtlesim_node',
    #     name='sim'
    # )
    
    driver_share_dir = ament_index_python.packages.get_package_share_directory('vinerobot_bringup')
    driver_params_file = os.path.join(driver_share_dir, 'config', 'VLP16-velodyne_driver_node-params.yaml')
    velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[driver_params_file])

    convert_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file = os.path.join(convert_share_dir, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(convert_share_dir, 'params', 'VLP16db.yaml')
    velodyne_transform_node = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                    executable='velodyne_transform_node',
                                                    output='both',
                                                    parameters=[convert_params])

    laserscan_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_laserscan')
    laserscan_params_file = os.path.join(laserscan_share_dir, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node = launch_ros.actions.Node(package='velodyne_laserscan',
                                                      executable='velodyne_laserscan_node',
                                                      output='both',
                                                      parameters=[laserscan_params_file])
    
    return launch.LaunchDescription([
        # oak_camera,
        # joy_node,
        # teleop_node,
        # turtlesim_node,
        velodyne_driver_node,
        velodyne_transform_node,
        velodyne_laserscan_node,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=velodyne_driver_node,
                on_exit=[launch.actions.EmitEvent(
                event=launch.events.Shutdown())],
            )),
    ])




if __name__ == '__main__':
    generate_launch_description()

