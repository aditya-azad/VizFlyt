import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vicon_receiver.msg import Position
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from Modules.Planner import Planner
from Modules.StudentPerception import StudentPerception
from Modules.StudentPlanning import StudentMotionPlanning
from rclpy.qos import QoSProfile, ReliabilityPolicy

class FakeDroneController(Node):
    """
    Simulated Drone Control Node that subscribes to Vicon position, RGB, and Depth images.
    It computes velocity commands using student-implemented Perception and Motion Planning,
    then publishes them to the /cmd_vel topic.
    """

    def __init__(self):
        super().__init__('fake_drone_control_node')

        # QoS Profile to ensure best-effort communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscribers
        self.position_sub = self.create_subscription(
            Position, '/vicon/VizFlyt/VizFlyt', self.position_callback, qos_profile
        )
        self.rgb_sub = self.create_subscription(
            Image, '/vizflyt/rgb_image', self.rgb_callback, qos_profile
        )
        self.depth_sub = self.create_subscription(
            Image, '/vizflyt/depth_image', self.depth_callback, qos_profile
        )

        # Publisher for velocity commands
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Image processing
        self.bridge = CvBridge()

        # State variables
        self.current_pose = None
        self.current_rgb = None
        self.current_depth = None
        self.latest_timestamp = None 
        
        # Initialize student planner
        perception = StudentPerception()
        motion_planning = StudentMotionPlanning()
        self.planner = Planner(mode="velocity", perception_module=perception, motion_planning_module=motion_planning)

        self.get_logger().info("Fake Drone Control Node Initialized")

    def position_callback(self, msg):
        """Stores the latest drone position from Vicon and updates timestamp."""
        self.current_pose = msg
        # self.update_latest_timestamp(msg.header.stamp)

    def rgb_callback(self, msg):
        """Stores the latest RGB image and updates timestamp."""
        self.current_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.update_latest_timestamp(msg.header.stamp)

    def depth_callback(self, msg):
        """Stores the latest Depth image and updates timestamp."""
        self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.update_latest_timestamp(msg.header.stamp)

    def update_latest_timestamp(self, timestamp):
        """Updates latest timestamp based on incoming messages."""
        # if self.latest_timestamp is None or timestamp > self.latest_timestamp:
        self.latest_timestamp = timestamp

    def control_loop(self):
        """Processes inputs and publishes velocity commands."""
        if self.current_pose is None or self.current_rgb is None or self.current_depth is None:
            self.get_logger().info("Waiting for all sensor data...")
            return

        # Compute velocity or waypoint based on student logic
        cmd = self.planner.compute_command(self.current_rgb, self.current_depth)

        # Publish velocity command
        twist = Twist()
        
        if self.planner.mode == "velocity":
            twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z = cmd
        else:
            self.get_logger().warn("Planner is in position mode, but Fake Drone only supports velocity commands.")

        self.vel_publisher.publish(twist)

        self.get_logger().info(
            f"Published Velocity: {twist.linear.x:.2f}, {twist.linear.y:.2f}, {twist.linear.z:.2f} "
            f"at Timestamp: {self.latest_timestamp}"
        )

def main():
    """Main function."""
    rclpy.init()
    node = FakeDroneController()
    timer = node.create_timer(0.1, node.control_loop)  # Run control loop at 10Hz
    rclpy.spin(node)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
