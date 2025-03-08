#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Fake Vicon Node (Keyboard-based simulation)
    fake_vicon_node_keyboard = TimerAction(
        period=2.0,  # Delay of 2 seconds
        actions=[Node(
            package='vizflyt',
            executable=os.path.join(os.getcwd(), 'vizflyt/vizflyt/fake_vicon_node_keyboard.py'),
            name='fake_vicon_node_keyboard',
            output='screen',
            prefix=['python3']
        )]
    )

    # Fake Vicon Node (HITL mode)
    fake_vicon_node_hitl = TimerAction(
        period=4.0,  # Delay of 4 seconds
        actions=[Node(
            package='vizflyt',
            executable=os.path.join(os.getcwd(), 'vizflyt/vizflyt/fake_vicon_node_hitl.py'),
            name='fake_vicon_node_hitl',
            output='screen',
            prefix=['python3']
        )]
    )

    # Render Node
    render_node = TimerAction(
        period=6.0,  # Delay of 6 seconds
        actions=[Node(
            package='vizflyt',
            executable=os.path.join(os.getcwd(), 'vizflyt/vizflyt/render_node.py'),
            name='render_node',
            output='screen',
            prefix=['python3'],
            arguments=[],
        )]
    )

    # Collision Detection Node
    collision_detection_node = TimerAction(
        period=8.0,  # Delay of 8 seconds
        actions=[Node(
            package='vizflyt',
            executable=os.path.join(os.getcwd(), 'vizflyt/vizflyt/collision_detection_node.py'),
            name='collision_detection_node',
            output='screen',
            prefix=['python3'],
        )]
    )

    return LaunchDescription([
        fake_vicon_node_keyboard,
        fake_vicon_node_hitl,
        render_node, 
        collision_detection_node,
    ])
