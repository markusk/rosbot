#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Motor Controller Node
        Node(
            package='rosbot',
            executable='MotorControllerNode',
            name='motor_controller',
            output='screen',
            emulate_tty=True,
        ),
        
        # LED PWM Node (optional)
        Node(
            package='rosbot',
            executable='LEDPWMNode',
            name='led_pwm',
            output='screen',
            emulate_tty=True,
        ),
    ])
