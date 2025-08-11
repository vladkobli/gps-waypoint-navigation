from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory(
        "nav2_gps_waypoint_follower_demo")
    rl_params_file = os.path.join(
        gps_wpf_dir, "config", "dual_ekf_navsat_params.yaml")

    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                "output_final_position", default_value="false"
            ),
            launch.actions.DeclareLaunchArgument(
                "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
            ),
            # EKF fusing wheels+IMU → local odometry
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[rl_params_file],
                remappings=[
                    ('odometry/wheel',       '/panther/odometry/wheels'),
                    ('imu/data',             '/panther/imu_broadcaster/imu'),
                    ('odometry/filtered',    'odometry/local'),
                ],
            ),
            # EKF fusing local→global
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[rl_params_file],
                remappings=[
                    ('odometry/filtered', 'odometry/global'),
                ],
            ),
            # navsat_transform to tie GPS + IMU → global odom
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[rl_params_file],
                remappings=[
                    ('imu/data',        '/panther/imu_broadcaster/imu'),
                    ('gps/fix',         '/ublox_gps_node/fix'),
                    ('gps/filtered',    'gps/filtered'),
                    ('odometry/gps',    'odometry/gps'),
                    ('odometry/filtered','odometry/local'),
                ],
            ),
            # # (Your waypoint-follower / nav2 controller node)
            # launch_ros.actions.Node(
            #     package="nav2_gps_waypoint_follower_demo",
            #     executable="gps_waypoint_follower",
            #     name="gps_waypoint_follower",
            #     output="screen",
            #     remappings=[
            #         ('cmd_vel', '/panther/cmd_vel'),
            #     ],
            # ),
        ]
    )
