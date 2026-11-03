"""Real robot (ROS 2 Galactic): cameras + lane node (on /cmd_vel_lane) + cmd_vel mux + mission FSM.

  On the robot:  ros2 launch turtlebot3_bringup robot.launch.py          # /odom, /scan, takes /cmd_vel
  Terminal 2:    ros2 launch mission_model mission_model.launch.py        # senior's sign detector (/camera2)
  Terminal 3:    ros2 launch autorace_mission mission_robot.launch.py

camera_ros (vendored in lane_detection_ad, the senior's code): publisher -> /camera (lane, undistorted),
publisher2 -> /camera2 (front camera, /dev/video2). Set cameras:=false if they already run.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        DeclareLaunchArgument('start_mode', default_value='light', description='light | go'),
        DeclareLaunchArgument('v_max', default_value='0.08', description='m/s, first real runs: keep low'),
        DeclareLaunchArgument('show', default_value='false', description='lane debug window (needs a display)'),
        DeclareLaunchArgument('cameras', default_value='true', description='start camera_ros publisher + publisher2'),
        Node(
            package='camera_ros', executable='publisher', name='camera_publisher', output='screen',
            condition=IfCondition(arg('cameras')),
        ),
        Node(
            package='camera_ros', executable='publisher2', name='camera_publisher2', output='screen',
            condition=IfCondition(arg('cameras')),
        ),
        Node(
            package='lane_detection_ad', executable='detection', name='lane_detection_ad', output='screen',
            parameters=[arg('lane_params'), {
                'start_enabled': False,
                'cmd_vel_topic': '/cmd_vel_lane',
                'show': ParameterValue(arg('show'), value_type=bool),
                'v_max': ParameterValue(arg('v_max'), value_type=float),
            }],
        ),
        Node(
            package='autorace_mission', executable='cmd_vel_mux', name='cmd_vel_mux', output='screen',
            parameters=[arg('mission_params')],
        ),
        Node(
            package='autorace_mission', executable='mission_fsm', name='mission_fsm', output='screen',
            parameters=[arg('mission_params'), {'start_mode': arg('start_mode')}],
        ),
    ])
