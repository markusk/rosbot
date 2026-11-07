#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Describe the core robot nodes and optional joystick input."""
    joystick_enabled = LaunchConfiguration('joystick_enabled')
    joy_device_id = LaunchConfiguration('joy_device_id')
    nodes = [
        DeclareLaunchArgument('joystick_enabled', default_value='false'),
        DeclareLaunchArgument('joy_device_id', default_value='0'),
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
        Node(
            package='rosbot',
            executable='TeleopKeyboardNode',
            name='teleop_keyboard',
            output='screen',
            emulate_tty=True,
        ),
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

    # if socket.gethostname() == 'rosbot':
    #     nodes.append(
    #         Node(
    #             package='joy',
    #             executable='joy_node',
    #             name='joy_node',
    #             parameters=[{
    #                 'device_id': ParameterValue(joy_device_id, value_type=int),
    #             }],
    #             condition=IfCondition(joystick_enabled),
    #             output='screen',
    #         )
    #     )

    return LaunchDescription(nodes)
