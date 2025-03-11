"""
Reads Pose from the 'drone' frame and re-publishes it as 'Position' message which is used by our 
external localzization setup (Vicon). Hence, this is a fake vicon node that will take Pose from fake 'drone' frame.   
"""
import rclpy
import math
from rclpy.node import Node
from vicon_receiver.msg import Position
from tf2_ros import Buffer, TransformListener

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('fake_vicon_node_hitl')

        self.publisher_ = self.create_publisher(Position, '/vicon/VizFlyt/VizFlyt', 10)
        
        # TF listener to get transformations from RVIZ
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer_period = 0.005  # 200Hz
        self.timer = self.create_timer(self.timer_period, self.publish_trajectory)        
        self.get_logger().info("Fake Vicon Node Initialized, receiving transform data")

    def publish_trajectory(self):
        """Obtain position from TF and publish position messages in NWU frame."""
        try:
            trans = self.tf_buffer.lookup_transform('map', 'fake_drone', rclpy.time.Time())
            
            msg = Position()
            msg.x_trans = trans.transform.translation.x  
            msg.y_trans = trans.transform.translation.y
            msg.z_trans = trans.transform.translation.z
            
            msg.x_rot = trans.transform.rotation.x
            msg.y_rot = trans.transform.rotation.y
            msg.z_rot = trans.transform.rotation.z
            msg.w = trans.transform.rotation.w
            
            self.publisher_.publish(msg)
            
            # self.get_logger().info(f'Published Position: x={msg.x_trans:.2f}, y={msg.y_trans:.2f}, z={msg.z_trans:.2f}')
        except Exception as e:
            self.get_logger().warn(f'Failed to get transform: {e}')

def main():
    rclpy.init()
    node = PositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
