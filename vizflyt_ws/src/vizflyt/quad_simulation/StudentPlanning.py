import numpy as np

class StudentMotionPlanning:
    """
    Implements a state machine to execute a predefined movement pattern:
    1. Take off to z=1m.
    2. Move forward by 1m.
    3. Move backward by 1m.
    4. Move left by 0.5m.
    5. Move right by 0.5m.
    """

    def __init__(self):
        self.state = 0  # Start with takeoff
        self.target_positions = [
            np.array([0.0, 0.0, 0.2]),  # Takeoff to 1m height
            np.array([0.5, 0.0, 0.2]),  # Move forward 1m
            np.array([0.4, -0.2, 0.2]),  # Move backward 1m
            np.array([0.0, -0.2, 0.2]), # Move left 0.5m
            np.array([0.0, 0.2, 0.2]),  # Move right 0.5m
            np.array([0.0, 0.0, 0.2]),
            np.array([0.0, 0.0, 0.0]),
        ]
        self.current_target = self.target_positions[self.state]
        self.threshold = 0.05  # Distance threshold to consider waypoint reached

    def update_target(self, current_pose):
        """
        Checks if the drone reached the target and updates the next waypoint.
        Args:
            current_pose: Current (x, y, z) position of the drone.
        Returns:
            Next target position (x, y, z)
        """
        distance = np.linalg.norm(self.current_target - current_pose)
        print(f"\n\nDistance : {distance}", f"\nThreshold : {self.threshold}")
        if distance < self.threshold and self.state < len(self.target_positions) - 1:
            self.state += 1
            self.current_target = self.target_positions[self.state]

        return self.current_target
