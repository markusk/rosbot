#!/usr/bin/env python3
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Describe the core robot nodes and optional joystick input."""
    joystick_enabled = LaunchConfiguration('joystick_enabled')
    joy_device_id = LaunchConfiguration('joy_device_id')
    camera_enabled = LaunchConfiguration('camera_enabled')
    laser_enabled = LaunchConfiguration('laser_enabled')
    laser_device = LaunchConfiguration('laser_device')
    laser_baud = LaunchConfiguration('laser_baud')
    laser_frame_id = LaunchConfiguration('laser_frame_id')
    keyboard_node = Node(
        package='rosbot',
        executable='TeleopKeyboardNode',
        name='teleop_keyboard',
        output='screen',
        emulate_tty=True,
    )
    nodes = [
        DeclareLaunchArgument('joystick_enabled', default_value='false'),
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        DeclareLaunchArgument('camera_enabled', default_value='false'),
        DeclareLaunchArgument('laser_enabled', default_value='false'),
        DeclareLaunchArgument('laser_device', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('laser_baud', default_value='115200'),
        DeclareLaunchArgument('laser_frame_id', default_value='laser'),
        RegisterEventHandler(
            OnProcessExit(
                target_action=keyboard_node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason='Keyboard teleoperation exited')),
                ],
            ),
        ),
        Node(
            package='rosbot',
            executable='MotorControllerNode',
            name='motor_controller',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='rosbot',
            executable='TeleopArbitratorNode',
            name='teleop_arbitrator',
            output='screen',
        ),
        keyboard_node,
        Node(
            package='rosbot',
            executable='TeleopJoystickNode',
            name='teleop_joystick',
            condition=IfCondition(joystick_enabled),
            output='screen',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': ParameterValue(joy_device_id, value_type=int),
            }],
            condition=IfCondition(joystick_enabled),
            output='screen',
        ),
    ]

    if socket.gethostname() == 'rosbot':
        nodes.append(
            Node(
                package='camera_ros',
                executable='camera_node',
                name='camera',
                condition=IfCondition(camera_enabled),
                output='screen',
            )
        )
        nodes.append(
            Node(
                package='urg_node',
                executable='urg_node_driver',
                name='urg_node',
                parameters=[{
                    'serial_port': laser_device,
                    'serial_baud': ParameterValue(laser_baud, value_type=int),
                    'laser_frame_id': laser_frame_id,
                }],
                condition=IfCondition(laser_enabled),
                output='screen',
            )
        )

    return LaunchDescription(nodes)
