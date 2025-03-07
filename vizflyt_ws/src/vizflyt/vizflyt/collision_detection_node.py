import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

class CollisionDetectionNode(Node):
    def __init__(self):
        super().__init__("collision_detection_node")
        
        self.occupancy_grid = o3d.io.read_point_cloud("vizflyt_viewer/occupancy_grid/splat.ply")
        
        if not self.occupancy_grid.has_points():
            self.get_logger().error("Occupancy PLY file is empty or not loaded correctly")
            exit(1)
            
        # tunable params 
        self.collision_threshold = 0.01
        self.drone_radius = 0.01
        self.num_points = 10
        
        phi = np.linspace(0, np.pi, self.num_points)
        theta = np.linspace(0, 2 * np.pi, self.num_points)
        self.phi, self.theta = np.meshgrid(phi, theta)

        self.x = self.drone_radius * np.sin(self.phi) * np.cos(self.theta)
        self.y = self.drone_radius * np.sin(self.phi) * np.sin(self.theta)
        self.z = self.drone_radius * np.cos(self.phi)
                
        self.subscription = self.create_subscription(
            PointStamped,                
            "/vizflyt/splat_cam_location",   
            self.pose_callback,          
            10                           
        )
        
        # self.stop_publisher = self.create_publisher(Bool, "/vizflyt/stop_render", 10)
        self.stop_render_client = self.create_client(Trigger, "/vizflyt/stop_render")
        
        self.get_logger().info("Collision Detection Node Initialized")
        
        
    def pose_callback(self, msg:PointStamped):
        """
        Receives splatcam location from the Digital Twin and checks for collisions
        """
        
        x,y,z = msg.point.x, msg.point.y, msg.point.z 
        current_position = np.array([x,y,z])
        
        if self.collision_check(current_position):
            self.get_logger().warn("Collision Warning: Drone is too close to an obstacle!")

            # # The Pub-Sub way to terminate rendering
            # stop_msg = Bool()
            # stop_msg.data = True
            # self.stop_publisher.publish(stop_msg)            
            
            # The server-client wat to terminate rendering
            if self.stop_render_client.wait_for_service(timeout_sec=2.0):
                request = Trigger.Request()
                future = self.stop_render_client.call_async(request)
                future.add_done_callback(self.service_response_callback)
            else:
                self.get_logger().error("Stop Render Service is not available")
    
    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Stop Render Service Response: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")    
                
            
    def collision_check(self, current_position):
        """
        Checks if the drone has collided with an obstacle inside the Digital Twin. 
        """    
        
        sphere_points = np.vstack((self.x.ravel(), self.y.ravel(), self.z.ravel())).T
        sphere_points += current_position.reshape(1, 3)
        pcd_robot = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(sphere_points))
        distances = np.array(self.occupancy_grid.compute_point_cloud_distance(pcd_robot))
        
        if distances.size == 0:
            self.get_logger().warn("No points found in occupancy grid for distance check.")
            return False  # No obstacles detected
        
        # self.get_logger().info(f"Min. Distance: {min(distances)}, ")
        
        return np.any(distances < self.collision_threshold)
    
    
def main():
    
    rclpy.init()
    node = CollisionDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__=="__main__":
    main()
    
    


        


        
        
        
        

        
    