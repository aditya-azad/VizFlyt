#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Render Node
    render_node = TimerAction(
        period=0.5,  # Delay of 6 seconds
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
        period=10.0,  # Delay of 8 seconds
        actions=[Node(
            package='vizflyt',
            executable=os.path.join(os.getcwd(), 'vizflyt/vizflyt/collision_detection_node.py'),
            name='collision_detection_node',
            output='screen',
            prefix=['python3'],
        )]
    )

    # Fake Vicon Node (HITL mode)
    fake_vicon_node_hitl = TimerAction(
        period=8.0,  # Delay of 4 seconds
        actions=[Node(
            package='vizflyt',
            executable=os.path.join(os.getcwd(), 'vizflyt/vizflyt/fake_vicon_node_hitl.py'),
            name='fake_vicon_node_hitl',
            output='screen',
            prefix=['python3']
        )]
    )
    
    quad_simulator_node = TimerAction(
        period=10.0,
        actions=[Node(
            package='vizflyt',
            executable=os.path.join(os.getcwd(), 'vizflyt/quad_simulation/quad_simulator_node.py'),
            name='quad_simulator_node',
            output='screen',
            prefix=['python3']
        )]
    )
    
    usercode_node = TimerAction(
        period=12.0,
        actions=[Node(
            package='vizflyt',
            executable=os.path.join(os.getcwd(), 'vizflyt/quad_simulation/usercode_node.py'),
            name='usercode_node',
            output='screen',
            prefix=['python3']
        )]
    )

    
    return LaunchDescription([
        render_node, 
        collision_detection_node,
        # fake_vicon_node_hitl,
        # quad_simulator_node,
        # usercode_node,
    ])
