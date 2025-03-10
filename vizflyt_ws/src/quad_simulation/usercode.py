import numpy as np
import scipy.io

class StateMachines:
    """
    
    """
    def __init__(self):
        
        self.dt = 0.1
        self.MP = np.genfromtxt(
            fname="quad_simulation/traj/MP_NWU.csv",
            delimiter=',',
            skip_header=0,
        )
        self.active_index = 0
        self.max_index = self.MP.shape[1]
        
    def step(self, time, current_pose, current_velocity):
        """
        Stepping through the trajectory
        """    
        
        if (self.active_index >= self.max_index):
            
            self.active_index = self.max_index -1 # stay at the last point
        
        
        xyz_desired = self.MP[:3, self.active_index]
        vel_desired = self.MP[3:6, self.active_index]
        acc_desired = self.MP[6:, self.active_index]
        yaw_setpoint = 0.0
        
        self.active_index += 1
        
        # print(xyz_desired)
        
        return xyz_desired, vel_desired, acc_desired, yaw_setpoint

