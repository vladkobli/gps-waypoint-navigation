import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    gps_wpf_dir = get_package_share_directory(
        "nav2_gps_waypoint_follower_demo")
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    params_dir = os.path.join(gps_wpf_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )


    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dual_ekf_navsat.launch.py'))
    )
    
    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )
    
    waypoints_arg = DeclareLaunchArgument(
        "waypoints_file",
        default_value=os.path.join(params_dir, "waypoints.yaml"),
        description="Path to GPS waypoints YAML"
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # robot localization launch
    ld.add_action(robot_localization_cmd)

    ######
    # ld.add_action(Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='odom_to_base_link',
    #     arguments=['0','0','0', '0','0','0', 'odom', 'base_link']
    # ))
    ######
    
    # navigation2 launch
    ld.add_action(navigation2_cmd)
    
    ld.add_action(waypoints_arg)

    ld.add_action(Node(
        package='nav2_gps_waypoint_follower_demo',
        executable='logged_waypoint_follower',
        name='logged_waypoint_follower',
        arguments=[LaunchConfiguration('waypoints_file')],
        output='screen'
    ))
    
    return ld
