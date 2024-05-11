# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'joy_teleop.yaml'
    )
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml'
    )
    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'sensors.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )
    odom_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'ekf.yaml'
    )

    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs')
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')
    sensors_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config,
        description='Descriptions for sensor configs')
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')
    odom_la = DeclareLaunchArgument(
        'odom_config',
        default_value=odom_config,
        description='Descriptions for robot localization configs')

    ld = LaunchDescription([joy_la, vesc_la, sensors_la, mux_la, odom_la])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    throttle_interpolator_node = Node(
        package='f1tenth_stack',
        executable='throttle_interpolator',
        name='throttle_interpolator',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[LaunchConfiguration('sensors_config')]
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_drive_out', 'ackermann_cmd')]
    )
    twist_stamp_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        remappings=[('cmd_vel_in', 'cmd_vel'), ('cmd_vel_out', 'cmd_twist')],
        parameters=[{'frame_id': 'map'}]
    )
    twist_to_ackermann_node = Node(
        package='twist_to_ackermann',
        executable='twist_to_ackermann',
        remappings=[('nav_vel', 'cmd_twist'), ('ack_vel', 'drive')],
        parameters=[{'wheelbase': 0.345}, {'use_stamps': True}]
    )
    # static_tf_node1 = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_basefootprint_to_baselink',
    #     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'base_link']
    # )
    static_tf_node2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )
    static_tf_node3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        arguments=['0.07', '0.0', '0.05', '0.0', '0.0', '0.7071068', '0.7071068', 'base_link', 'imu']
    )
    # rf2o_laser_odometry_node = Node(
    #     package='rf2o_laser_odometry',
    #     executable='rf2o_laser_odometry_node',
    #     name='rf2o_laser_odometry',
    #     output='log',
    #     parameters=[{
    #         'laser_scan_topic' : '/scan',
    #         'odom_topic' : '/odom_rf2o',
    #         'publish_tf' : False,
    #         'base_frame_id' : 'base_footprint',
    #         'odom_frame_id' : 'odom',
    #         'init_pose_from_topic' : '',
    #         'freq' : 50.0}],
    #     # arguments=['--ros-args', '--log-level', 'debug'],
    # )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[LaunchConfiguration('odom_config')],
        remappings=[('odometry/filtered', 'odom')]
    )



    # finalize
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    # ld.add_action(throttle_interpolator_node)
    ld.add_action(urg_node)
    ld.add_action(ackermann_mux_node)
    # ld.add_action(static_tf_node1)
    ld.add_action(static_tf_node2)
    ld.add_action(static_tf_node3)
    ld.add_action(twist_stamp_node)
    ld.add_action(twist_to_ackermann_node)
    # ld.add_action(rf2o_laser_odometry_node)
    ld.add_action(robot_localization_node)

    return ld
