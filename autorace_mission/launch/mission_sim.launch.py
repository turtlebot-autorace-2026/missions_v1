"""Gazebo: lane camera bridge + lane node (on /cmd_vel_lane) + cmd_vel mux + mission FSM, sim time.

Terminal 1: ros2 launch autorace_gazebo autorace.launch.py
Terminal 2: ros2 launch mission_model mission_model.launch.py sim:=true      # senior's sign detector
Terminal 3: ros2 launch autorace_mission mission_sim.launch.py

The traffic light check needs the front camera on /camera2. If your Gazebo robot has no front camera,
start with start_mode:=go. If the senior's camera_ros has no sim_camera node, front_bridge:=true makes
/camera2 from /front_camera/image_raw here instead (do not use both).
Signs by hand (no detector): ros2 topic pub --once /intersection std_msgs/msg/Bool "{data: true}"
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
    sim = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('lane_params', default_value=os.path.join(lane_share, 'config', 'lane_params.yaml')),
        DeclareLaunchArgument('mission_params', default_value=os.path.join(share, 'config', 'mission_params.yaml')),
        DeclareLaunchArgument('start_mode', default_value='light', description='light | go'),
        DeclareLaunchArgument('v_max', default_value='0.12'),
        DeclareLaunchArgument('show', default_value='true'),
        DeclareLaunchArgument('front_bridge', default_value='false',
                              description='true: /front_camera/image_raw -> /camera2 here'),
        Node(
            package='lane_detection_ad', executable='sim_camera_bridge', name='sim_camera_bridge',
            output='screen', parameters=[sim],
        ),
        Node(
            package='lane_detection_ad', executable='sim_camera_bridge', name='front_camera_bridge',
            output='screen', condition=IfCondition(arg('front_bridge')),
            parameters=[sim, {'in_topic': '/front_camera/image_raw', 'out_topic': '/camera2'}],
        ),
        Node(
            package='lane_detection_ad', executable='detection', name='lane_detection_ad', output='screen',
            parameters=[arg('lane_params'), sim, {
                'start_enabled': False,
                'cmd_vel_topic': '/cmd_vel_lane',
                'show': ParameterValue(arg('show'), value_type=bool),
                'v_max': ParameterValue(arg('v_max'), value_type=float),
            }],
        ),
        Node(
            package='autorace_mission', executable='cmd_vel_mux', name='cmd_vel_mux', output='screen',
            parameters=[arg('mission_params'), sim],
        ),
        Node(
            package='autorace_mission', executable='mission_fsm', name='mission_fsm', output='screen',
            parameters=[arg('mission_params'), sim, {'start_mode': arg('start_mode')}],
        ),
    ])
