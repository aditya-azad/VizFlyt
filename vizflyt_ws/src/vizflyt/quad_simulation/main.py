import numpy as np
import scipy.io
from usercode import StateMachines
from control import QuadControl
import quad_dynamics as qd
import tello

def main():
    # Simulation Parameters
    sim_stop_time = 100.0
    dynamics_dt = 0.001

    # Initialize controller and trajectory generator
    controller = QuadControl()
    user_state_machine = StateMachines()

    # Initial drone state: [x, y, z, vx, vy, vz, qx, qy, qz, qw, p, q, r]
    current_state = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], dtype=np.float64)

    # Initialize time variables
    current_time = 0.0
    control_timer, user_timer = 0.0, 0.0
    control_dt = controller.dt
    user_dt = user_state_machine.dt

    # Logging
    pose_log = []
    time_log = []

    position_SP = np.array([0, 0, 0, 0])
    velocity_SP = np.array([0, 0, 0])
    acceleration_SP = np.array([0, 0, 0])

    # Simulation loop
    while current_time < sim_stop_time:
        if user_timer <= 0:
            pos_desired, vel_desired, acc_desired, yaw_desired = user_state_machine.step(
                current_time,
                current_state[:3],
                current_state[3:6]
            )

            position_SP = np.array([pos_desired[0], pos_desired[1], pos_desired[2], yaw_desired])
            velocity_SP = vel_desired
            acceleration_SP = acc_desired
            user_timer = user_dt

        if control_timer <= 0:
            U = controller.step(current_state, position_SP, velocity_SP, acceleration_SP)
            control_timer = control_dt

        # Dynamics simulation step
        current_state += dynamics_dt * qd.model_derivative(
            current_time,
            current_state,
            U,
            tello,
        )

        # Log data
        pose_log.append(np.hstack((current_state[:3], current_state[6:10])))
        time_log.append(current_time)

        # Update timers
        control_timer -= dynamics_dt
        user_timer -= dynamics_dt
        current_time += dynamics_dt

    # Save logged data
    logged_data = {
        'time': np.array(time_log),
        'position': np.array(pose_log)[:, :3],
        'orientation_quat': np.array([p[3:] for p in pose_log])
    }

    

    scipy.io.savemat('vizflyt/quad_simulation/log/quadrotor_pose.mat', logged_data)
    print("Simulation completed. Results saved to './vizflyt/quad_simulation/log/quadrotor_pose.mat'.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nSimulation terminated by user (Ctrl+C).")
