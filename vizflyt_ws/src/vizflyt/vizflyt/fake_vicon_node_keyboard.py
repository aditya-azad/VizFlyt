import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

def normalize_angle(angle):
    """ Normalize angle to the range [-pi, pi] """
    return math.atan2(math.sin(angle), math.cos(angle))

class TurtleBotTFPublisher(Node):
    def __init__(self):
        super().__init__('fake_vicon_node_keyboard')
        
        # Initialize position and orientation
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta_x = 0.0  # Roll angle
        self.theta_y = 0.0  # Pitch angle
        self.theta_z = 0.0  # Yaw angle
        
        # Create a subscriber to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Create a TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to update transform at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_transform)
        
        self.last_time = self.get_clock().now()
    
    def cmd_vel_callback(self, msg):
        """ Callback function for cmd_vel messages """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        self.last_time = current_time

        # Update position using simple integration
        self.x += msg.linear.x * dt
        self.y += msg.linear.y * dt
        self.z += msg.linear.z * dt
        
        # Update orientation angles
        self.theta_x += msg.angular.x * dt
        self.theta_y += msg.angular.y * dt
        self.theta_z += msg.angular.z * dt
        
        # Normalize angles
        self.theta_x = normalize_angle(self.theta_x)
        self.theta_y = normalize_angle(self.theta_y)
        self.theta_z = normalize_angle(self.theta_z)

    def publish_transform(self):
        """ Publishes the transform of the TurtleBot """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'  # Fixed frame
        t.child_frame_id = 'turtlebot_base'  # TurtleBot frame
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        
        # Convert roll, pitch, yaw to quaternion
        cy = math.cos(self.theta_z * 0.5)
        sy = math.sin(self.theta_z * 0.5)
        cp = math.cos(self.theta_y * 0.5)
        sp = math.sin(self.theta_y * 0.5)
        cr = math.cos(self.theta_x * 0.5)
        sr = math.sin(self.theta_x * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)
        
        # self.get_logger().info(f'Published TF: x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, roll={math.degrees(self.theta_x):.2f}°, pitch={math.degrees(self.theta_y):.2f}°, yaw={math.degrees(self.theta_z):.2f}°')


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
