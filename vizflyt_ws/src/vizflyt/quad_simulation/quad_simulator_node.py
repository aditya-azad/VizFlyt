#!/usr/bin/env python3

"""A drone simulator that takes in trajectory (pos, vel, acc, yaw) from the user, and then publishes Quadrotor's State
   after going through the simulator
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import tf2_ros
# from quad_simulation.usercode_copy import StateMachines
from quad_simulation.control import QuadControl
import quad_simulation.quad_dynamics as qd
import quad_simulation.tello as tello 
from std_msgs.msg import Float32MultiArray


class QuadSimulator(Node):
    def __init__(self):
        super().__init__('quad_simulator_node')

        # Pose Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/fake_drone/pose', 10)

        # TF Broadcaster for /tf
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribe to trajectory setpoints
        self.trajectory_sub = self.create_subscription(
            Float32MultiArray, '/fake_drone/trajectory_setpoint', self.trajectory_callback, 10
        )

        # Simulation parameters
        self.sim_stop_time = 100.0
        self.dynamics_dt = 0.001    

        # Components
        self.controller = QuadControl()
        # self.user_state_machine = StateMachines()

        # State initialization        [x,  y,  z, vx, vy, vz, qw, qx, qy, qz,  p,  q,  r ]
        self.current_state = np.array([0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.])
        self.current_time = 0.0

        # Setpoint initialization
        self.position_SP = np.array([0., 0., 0., 0.])
        self.velocity_SP = np.zeros(3)
        self.acceleration_SP = np.zeros(3)

        # Timers for simulation
        self.control_timer = 0.0
        self.user_timer = 0.0
        self.control_dt = self.controller.dt
        # self.user_dt = self.user_state_machine.dt

        # Control input initialization
        self.U = np.zeros(4)

        # Create ROS timer
        self.timer = self.create_timer(self.dynamics_dt, self.simulate_step)

        self.get_logger().info("Quad Simulator Node initialized and running.")

    def trajectory_callback(self, msg):
        """ Updates the drone's trajectory setpoints from the published trajectory command. """
        self.position_SP = np.array([msg.data[0], -msg.data[1], -msg.data[2], -msg.data[9]])
        self.velocity_SP = np.array([msg.data[3], -msg.data[4], -msg.data[5]])
        self.acceleration_SP = np.array([msg.data[6], -msg.data[7], -msg.data[8]])
        self.get_logger().info(f"Updated Trajectory Setpoint: {self.position_SP}")

    def simulate_control(self):
        """
        """
        if self.control_timer <= 0.0:
            self.U = self.controller.step(self.current_state, self.position_SP, self.velocity_SP, self.acceleration_SP)
            self.control_timer = self.controller.dt
        self.control_timer -= self.dynamics_dt

    # def simulate_user_input(self):
    #     """
    #     """
    #     if self.user_timer <= 0.0:
    #         pos_des, vel_des, acc_des, yaw_des = self.user_state_machine.step(
    #             self.current_time,
    #             self.current_state[:3],
    #             self.current_state[3:6]
    #         )
    #         self.position_SP = np.array([pos_des[0], -pos_des[1], -pos_des[2], -yaw_des])
    #         self.velocity_SP = np.array([vel_des[0], -vel_des[1], -vel_des[2]])
    #         self.acceleration_SP = np.array([acc_des[0], -acc_des[1], -acc_des[2]])
    #         self.user_timer = self.user_state_machine.dt
    #     self.user_timer -= self.dynamics_dt

    def simulate_dynamics(self):
        self.current_state += self.dynamics_dt * qd.model_derivative(
            self.current_time,
            self.current_state,
            self.U,
            tello
        )
        
    def publish_pose(self):
        """
        publishes poses for state feedback
        """        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = self.current_state[0] 
        pose_msg.pose.position.y = -self.current_state[1] 
        pose_msg.pose.position.z = -self.current_state[2]

        pose_msg.pose.orientation.x = self.current_state[7]
        pose_msg.pose.orientation.y = -self.current_state[8]
        pose_msg.pose.orientation.z = -self.current_state[9]
        pose_msg.pose.orientation.w = self.current_state[6]

        self.pose_pub.publish(pose_msg)

        # Publish TF transform
        self.publish_tf_transform()


    def publish_tf_transform(self):
        """
        publishes transforms for RVIZ Visualization
        """
        t = TransformStamped()

        # Set timestamp and frame info
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"      # The world frame
        t.child_frame_id = "fake_drone"     # The drone's frame

        # Set translation (position of the drone) 
        t.transform.translation.x =  self.current_state[0] 
        t.transform.translation.y = -self.current_state[1] 
        t.transform.translation.z = -self.current_state[2] 

        # Set rotation (orientation of the drone)
        t.transform.rotation.x =  self.current_state[7]
        t.transform.rotation.y = -self.current_state[8]
        t.transform.rotation.z = -self.current_state[9]
        t.transform.rotation.w =  self.current_state[6]

        # Publish transform
        self.tf_broadcaster.sendTransform(t)

    def simulate_step(self):
        """
        """
        # self.simulate_user_input()
        self.simulate_control()
        self.simulate_dynamics()
        self.publish_pose()

        if self.current_time >= self.sim_stop_time:
            self.get_logger().info("Simulation completed.")
            rclpy.shutdown()

        self.current_time += self.dynamics_dt
        self.control_timer -= self.dynamics_dt
        self.user_timer -= self.dynamics_dt

def main(args=None):
    rclpy.init(args=args)
    node = QuadSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Interrupted by user (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
