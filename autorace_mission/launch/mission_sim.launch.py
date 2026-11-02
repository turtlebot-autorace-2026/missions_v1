"""Gazebo: camera bridge + lane node + cmd_vel mux + mission FSM, on simulation time.

Terminal 1: ros2 launch autorace_gazebo autorace.launch.py referee:=false
Terminal 2: ros2 launch autorace_mission mission_sim.launch.py
Signs (until the detector exists): ros2 topic pub --once /left std_msgs/msg/Bool "{data: true}"

The lane node publishes /cmd_vel_lane (not cmd_vel) and waits for /lane_state from the FSM.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lane_share = get_package_share_directory('lane_detection_ad')
    share = get_package_share_directory('autorace_mission')
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('lane_params', default_value=os.path.join(lane_share, 'config', 'lane_params.yaml')),
        DeclareLaunchArgument('mission_params', default_value=os.path.join(share, 'config', 'mission_params.yaml')),
        DeclareLaunchArgument('v_max', default_value='0.12'),
        DeclareLaunchArgument('show', default_value='true'),
        DeclareLaunchArgument('left_topic', default_value='/left'),
        DeclareLaunchArgument('right_topic', default_value='/right'),
        Node(
            package='lane_detection_ad', executable='sim_camera_bridge', name='sim_camera_bridge',
            output='screen', parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='lane_detection_ad', executable='detection', name='lane_detection_ad', output='screen',
            parameters=[arg('lane_params'), {
                'use_sim_time': True,
                'start_enabled': False,
                'cmd_vel_topic': '/cmd_vel_lane',
                'show': ParameterValue(arg('show'), value_type=bool),
                'v_max': ParameterValue(arg('v_max'), value_type=float),
            }],
        ),
        Node(
            package='autorace_mission', executable='cmd_vel_mux', name='cmd_vel_mux', output='screen',
            parameters=[arg('mission_params'), {'use_sim_time': True}],
        ),
        Node(
            package='autorace_mission', executable='mission_fsm', name='mission_fsm', output='screen',
            parameters=[arg('mission_params'), {
                'use_sim_time': True,
                'left_topic': arg('left_topic'),
                'right_topic': arg('right_topic'),
            }],
        ),
    ])
