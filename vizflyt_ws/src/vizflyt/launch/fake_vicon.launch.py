from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Define Default Paths
    rviz_config_path = os.path.join(
        get_package_share_directory('vizflyt'), 'rviz', 'vizflyt.rviz'
    )

    # Fake Vicon Node (with localization from simulated drone travelling in a pre-defined path) 
    fake_vicon_node = Node(
        package='vizflyt',
        executable=os.path.join(os.getcwd(), 'vizflyt/vizflyt/fake_vicon_node.py'),
        name='fake_vicon_node',
        output='screen',
        prefix=['python3']
    )
    
    # Render Node
    render_node = Node(
        package='vizflyt',
        executable=os.path.join(os.getcwd(), 'vizflyt/vizflyt/render_node.py'),
        name='render_node',
        output='screen',
        prefix=['python3'],
        arguments=[],
    )
    
    # Collision Detection Node
    collision_detection_node = Node(
        package='vizflyt',
        executable=os.path.join(os.getcwd(), 'vizflyt/vizflyt/collision_detection_node.py'),
        name='collision_detection_node',
        output='screen',
        prefix=['python3'],
    )
    
    return LaunchDescription([
        
        fake_vicon_node,
        render_node, 
        collision_detection_node,
    ])
