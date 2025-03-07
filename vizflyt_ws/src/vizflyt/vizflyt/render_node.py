from __future__ import annotations

import os
import time
import numpy as np
import cv2
import torch
import sys
import argparse
import json

import rclpy
from rclpy.node import Node
from vicon_receiver.msg import Position 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PointStamped
from std_srvs.srv import Trigger
from transforms3d.euler import euler2quat

from dataclasses import dataclass, fields
from pathlib import Path

import tyro
from nerfstudio.configs.base_config import ViewerConfig
from nerfstudio.engine.trainer import TrainerConfig
from nerfstudio.pipelines.base_pipeline import Pipeline
from nerfstudio.utils.eval_utils import eval_setup
from nerfstudio.utils.colormaps import ColormapOptions
from nerfstudio.utils import colormaps
from nerfstudio.viewer.utils import CameraState, get_camera
from nerfstudio.utils.decorators import check_main_thread, decorate_all
from nerfstudio.cameras.cameras import CameraType

from vizflyt.utils import quaternion_to_euler, rotation_matrix_from_euler, quaternion_multiply, euler_to_quaternion


@dataclass
class ViewerConfigWithoutNumRays(ViewerConfig):
    """Configuration for viewer instantiation."""
    num_rays_per_chunk: tyro.conf.Suppress[int] = -1

    def as_viewer_config(self):
        """Converts the instance to ViewerConfig."""
        return ViewerConfig(**{x.name: getattr(self, x.name) for x in fields(self)})
    
    
@decorate_all([check_main_thread])
class RenderViews(Node):
    """
    Custom Viewer class for rendering images and depth maps using NeRFStudio.
    """
    config: TrainerConfig
    pipeline: Pipeline
    
    def __init__(self, config: TrainerConfig, pipeline: Pipeline, save_images:bool, json_path: str, aspect_ratio: float):
        super().__init__("render_node")

        self.config = config
        self.pipeline = pipeline
        self.model = self.pipeline.model
        self.background_color = torch.tensor([0.1490, 0.1647, 0.2157], device=self.model.device)
        
        # image settings
        self.aspect_ratio = aspect_ratio
        self.load_camera_settings(json_path)
        
        # colormap options
        self.colormap_options_rgb = ColormapOptions(colormap='default', normalize=True, colormap_min=0.0, colormap_max=1.0, invert=False)
        self.colormap_options_depth = ColormapOptions(colormap='gray', normalize=True, colormap_min=0.0, colormap_max=1.0, invert=False)
                       
        # Enable/Disable saving images
        self.save_images_to_disk = save_images
        
        if self.save_images_to_disk:
            self.setup_output_directories()
        
        self.image_id = 0
        self.render   = True # flag to control rendering   

        # cv bridge for publishing images
        self.bridge = CvBridge()

        # subscriber to the fake vicon node {The message type entirely depends on the localization module you have}
        self.subscription = self.create_subscription(Position, '/vicon/VizFlyt/VizFlyt', self.pose_callback, 10)
        
        # publishers (rgb, depth, drone_pose, drone location in GSplat)
        self.rgb_publisher            = self.create_publisher(Image, "/vizflyt/rgb_image", 10)
        self.depth_publisher          = self.create_publisher(Image, "/vizflyt/depth_image", 10)
        self.drone_pose_publisher     = self.create_publisher(PoseStamped, "/vizflyt/drone_pose_NED", 10)
        self.splat_cam_publisher      = self.create_publisher(PointStamped, "/vizflyt/splat_cam_location", 10)

        # service (to stop rendering when the drone crashes inside the Digital Twin)
        self.stop_render_service = self.create_service(Trigger, "/vizflyt/stop_render", self.stop_render_callback)

        # set origin is where you start in the environment; current keeps updating
        self.origin_pose = None
        self.current_pose = None
        
        # define a timer for render rate control
        self.render_timer = self.create_timer(0.033, self.timer_callback)
    
        self.get_logger().info("Custom Viewer Node Initialized")

    #########################
    # GENERAL UTILS METHODS
    #########################
    def load_camera_settings(self, json_path):
        """
        Loads camera settings from a json config file
        """
        try:
            
            # extract camera data from the json file
            with open(json_path, 'r') as f:
                camera_data = json.load(f)
        
            camera_matrix = np.array(camera_data["camera"]["c2w_matrix"])
            render_resolution = camera_data["camera"].get("render_resolution", 1080)
            fov_radians = camera_data["camera"].get("fov_radians", 1.3089969389957472)
            
            # assign image settings
            self.scale_ratio = 1.0
            self.max_res = render_resolution
            self.fov = fov_radians
            self.image_height = render_resolution
            self.image_width = int(self.image_height * self.aspect_ratio)
            self.depth_res = render_resolution
            self.init_position = camera_matrix[:, 3]
            self.init_orientation = camera_matrix[:, :3]
            pp_h = self.image_height / 2.0
            self.focal_length = pp_h / np.tan(self.fov / 2.0)
            self.fx = self.focal_length
            self.fy = self.focal_length
            self.cx = self.image_width / 2
            self.cy = self.image_height / 2
            
            print(f"Loaded camera settings from {json_path}")
            
        except Exception as e:
            
            print(f"Error loading camera settings {e}, setting default values")
            
            self.scale_ratio = 1
            self.max_res = 640
            self.image_height = 480
            self.image_width = 640
            self.depth_res = 640
            self.fov = 1.3089969389957472
            self.init_position = np.array([0, 0, 0])
            self.init_orientation = np.eye(3)  
            pp_h = self.image_height / 2.0
            self.focal_length = pp_h / np.tan(self.fov / 2.0)
            self.fx = self.focal_length
            self.fy = self.focal_length
            self.cx = self.image_width / 2
            self.cy = self.image_height / 2
            

    def setup_output_directories(self):
        """
        Creates directories for saving images, only if saving is enabled.
        """
        
        self.experiment_directory = "./ExpTest"
        os.makedirs(self.experiment_directory, exist_ok=True)

        # Determine the next output directory index
        subdirs = [d for d in os.listdir(self.experiment_directory) if os.path.isdir(os.path.join(self.experiment_directory, d))]
        output_dirs = [d for d in subdirs if d.startswith('output') and d[6:].isdigit()]
        highest_index = max([int(d[6:]) for d in output_dirs], default=0)
        
        self.output_directory = os.path.join(self.experiment_directory, f'output{highest_index + 1}')
        os.makedirs(self.output_directory)

        self.rgb_directory = os.path.join(self.output_directory, "rgb")
        self.depth_directory = os.path.join(self.output_directory, "depth")
        os.makedirs(self.rgb_directory, exist_ok=True)
        os.makedirs(self.depth_directory, exist_ok=True)

        print(f'Created new output directory: {self.output_directory}')
    
    def stop_render_callback(self, request, response):
        """
        Service callback to stop rendering
        """
        self.get_logger().warn("Stop Rendering Service called. Stopping Rendering...")
        self.render = False
        response.success = True
        response.message = "Rendering has been stopped!"
        
        return response
    
    
    def timer_callback(self):
        """
        Publishes rgb, depth, drone position in real world and drone_location in digital twin
        """
        
        if not self.render:
            return 
        
        if self.current_pose is None:
            return
        
        if self.current_pose is not None:
            
            # set origin pose if not already set
            if self.origin_pose is None:
                self.origin_pose = self.current_pose
            
            # convert from NWU to NED
            current_position_ned, current_orientation_ned = self.convert_to_ned(self.current_pose, self.origin_pose)

            # Render Images from the Splat        
            current_camera_state, current_rgb, current_depth = self.get_images(current_position_ned, current_orientation_ned)
            
            # prevent publishing empty message
            if current_rgb is None or current_depth is None:
                return
            
            # get timestamp
            current_time = self.get_clock().now().to_msg()

            # publish images and poses to topics
            self.publish_images(current_rgb, current_depth, current_time)
            self.publish_drone_pose(current_position_ned, current_orientation_ned, current_time)
            self.publish_splatcam_location(current_camera_state, current_time)
            
            # Saving images locally
            if self.save_images_to_disk:     
                cv2.imwrite(os.path.join(self.rgb_directory, f"{self.image_id:04d}.png"), current_rgb)
                cv2.imwrite(os.path.join(self.depth_directory, f"{self.image_id:04d}.png"), current_depth)
                self.image_id += 1
        
    def pose_callback(self, msg: Position):
        """
        Receives Vicon pose data, processes it, and saves an image.
        """
        # extract info from the fake node
        x, y, z = msg.x_trans, msg.y_trans, msg.z_trans
        x_quat, y_quat, z_quat, w_quat = msg.x_rot, msg.y_rot, msg.z_rot, msg.w
        
        # convert it into euler angles
        roll, pitch, yaw = quaternion_to_euler(x_quat, y_quat, z_quat, w_quat)
        self.current_pose = np.array([x, y, z, roll, pitch, yaw])
        
        
    def publish_images(self, rgb_image, depth_image, timestamp):
        """
        Publishes RGB and Depth images as ROS2 Image messages. 
        
        TODO: Currently these give out only RGB and Depth Images; 
              Modify this to publish desired type (eg. Events feed, stereo feed, etc.) 
        """
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8") 

        # RGB Image info 
        rgb_msg.header.stamp = timestamp
        rgb_msg.header.frame_id = "camera_frame"
        

        if len(depth_image.shape) == 3 and depth_image.shape[2] == 3:
            depth_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)  # Convert to grayscale if needed

        depth_image = depth_image.astype(np.float32)  # Convert to 32-bit float
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
        
        # Depth Image info
        depth_msg.header.stamp = timestamp  
        depth_msg.header.frame_id = "camera_frame"

        self.rgb_publisher.publish(rgb_msg)
        self.depth_publisher.publish(depth_msg)
        
        
    def publish_drone_pose(self, current_position, current_orientation, timestamp):
        """
        publish the current drone pose
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp  
        pose_msg.header.frame_id = "map"  # Use "map" or "world" as the global reference frame
        
        # Set position in NED frame
        pose_msg.pose.position.x = current_position[0]  # North
        pose_msg.pose.position.y = current_position[1]  # East
        pose_msg.pose.position.z = current_position[2]  # Down
        
        # Convert Euler angles (roll, pitch, yaw) to quaternion
        quat = euler2quat(
            current_orientation[0],  # Roll
            current_orientation[1],  # Pitch
            current_orientation[2]   # Yaw
        )
        
        # Set orientation in PoseStamped message
        pose_msg.pose.orientation.x = quat[1]
        pose_msg.pose.orientation.y = quat[2]
        pose_msg.pose.orientation.z = quat[3]
        pose_msg.pose.orientation.w = quat[0]

        # Publish pose
        self.drone_pose_publisher.publish(pose_msg)
        
    def publish_splatcam_location(self, camera_state, timestamp):
        """
        publishes camera to world transform from nerfstudio for collision detection node
        """        
        location_msg = PointStamped()
        location_msg.header.stamp = timestamp
        
        c2w = camera_state.c2w
        location_msg.point.x = c2w[0,3].item()
        location_msg.point.y = c2w[1,3].item()
        location_msg.point.z = c2w[2,3].item()
        
        self.splat_cam_publisher.publish(location_msg)
                    
    
    def convert_to_ned(self, current_pose, origin_pose):
        """
        convert from NWU to NED frame 
        """    
        x =  (current_pose[0] - origin_pose[0])    
        y = -(current_pose[1] - origin_pose[1])  
        z = -(current_pose[2] - origin_pose[2]) 
        roll = current_pose[3] - origin_pose[3]  
        pitch = -(current_pose[4] - origin_pose[4])  
        yaw = -(current_pose[5] - origin_pose[5])
        
        # Reason for current_pose - origin_pose -> To always initialize the drone from origin position and orientation
        
        return np.array([x,y,z]), np.array([roll, pitch, yaw])

    #########################
    # Nerfstudio Functions
    #########################
    def get_camera_state(self, rot_mat, pos):
        """
        Gets the state of the camera
        """
        rot_adjustment = np.eye(3)
        r = rotation_matrix_from_euler(np.radians(0), 0, 0)
        R = rot_adjustment @ rot_mat
        R = torch.tensor(r @ R)   
        pos = torch.tensor([pos[0], pos[1], pos[2]], dtype=torch.float64) / self.scale_ratio
        c2w = torch.concatenate([R, pos[:, None]], dim=1)
        camera_state = CameraState(fov=self.fov, aspect=self.aspect_ratio, c2w=c2w, camera_type=CameraType.PERSPECTIVE)
        return camera_state
    
    def _render_img(self, camera_state: CameraState):
        """
        Renders the RGB and depth images at the current camera frame
        """
        obb = None
        camera = get_camera(camera_state, self.image_height, self.image_width)
        camera = camera.to(self.model.device)
        self.model.set_background(self.background_color)
        self.model.eval()
        outputs = self.model.get_outputs_for_camera(camera, obb_box=obb)
        
        return outputs
    
    
    def current_images(self, outputs):
        """
        Returns RGB and Depth Image Outputs
        """
        rgb = outputs['rgb']
        depth = outputs["depth"]
                
        rgb_image = colormaps.apply_colormap(image=rgb, colormap_options=self.colormap_options_rgb)
        rgb_image = (rgb_image * 255).type(torch.uint8)
        rgb_image = rgb_image.cpu().numpy()
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        
        depth_image = colormaps.apply_colormap(image=depth, colormap_options=self.colormap_options_depth)
        depth_image = (depth_image * 255).type(torch.uint8)
        depth_image = depth_image.cpu().numpy()
        
        return rgb_image, depth_image

    #######################
    # EXECUTION FUNCTIONS
    #######################        
    def get_images(self, current_local_position_ned, current_local_orientation_ned):
        """
        Given Position and Orientation, return RGBD Images
        """
        # Computing position update within splat for drone control 
        # {NOTE: This is the current position the drone; called update because we'll add it to the Origin Position}
        position_update_cam = self.init_orientation @ np.array([[current_local_position_ned[1]],
                                                                [-current_local_position_ned[2]],
                                                                [-current_local_position_ned[0]]])
        
        # Updating the position in the splat for drone control loop
        current_position_cam = [self.init_position[0] + position_update_cam[0, 0],
                                self.init_position[1] + position_update_cam[1, 0],
                                self.init_position[2] + position_update_cam[2, 0]]
        
        # Updated Orientation in the nerfstudio cam frame
        r_cam = rotation_matrix_from_euler(current_local_orientation_ned[1],
                                            -current_local_orientation_ned[2],
                                            -current_local_orientation_ned[0])
        
        # Current Orientation with respect to the origin frame in nerfstudio
        current_orientation_cam = self.init_orientation @ r_cam
        
        # Getting camera state, rendering RGB and depth images at that camera state 
        camera_state = self.get_camera_state(current_orientation_cam, current_position_cam)
        outputs = self._render_img(camera_state)
        current_rgb, current_depth = self.current_images(outputs)
        
        return camera_state, current_rgb, current_depth 
    
def parse_args():
    """
    Parses command-line arguments for configuration path and aspect ratio.
    """
    parser = argparse.ArgumentParser(description="Render images from NeRFStudio and publish to ROS2 topics.")

    parser.add_argument(
        "--config", type=str, default="vizflyt_viewer/outputs/washburn-env6-itr0-1fps/washburn-env6-itr0-1fps_nf_format/splatfacto/2025-03-06_201843/config.yml", help="Path to config.yml (default: %(default)s)"
    )
    parser.add_argument(
        "--json", type=str, default="vizflyt_viewer/render_settings/render_config.json", help="Path to render_config.json (default: %(default)s)"
    )
    parser.add_argument(
        "--aspect-ratio", type=float, default=4/3, help="Aspect ratio (default: 4:3)"
    )
    parser.add_argument(
        "--save", action="store_true", help="Enable saving images to disk. If not set, images will only be published to ROS2."
    )
    args = parser.parse_args()

    return Path(args.config), Path(args.json), args.save, args.aspect_ratio

def main():
    
    rclpy.init()
    config_path, json_path, save_images, aspect_ratio = parse_args()
    config, pipeline, _, _ = eval_setup(config_path, eval_num_rays_per_chunk=None, test_mode="test")
    render_node = RenderViews(config, pipeline, save_images, json_path, aspect_ratio)
    rclpy.spin(render_node)
    render_node.destroy_node()
    rclpy.shutdown()    
    

if __name__ == "__main__":
    main()