class StudentMotionPlanning:
    """
    Sample student-implemented motion planning module.
    Generates velocity or waypoint commands based on perception output.
    """

    def compute_velocity(self, perception_output):
        """
        Compute velocity commands based on perception output.
        - Moves forward at 0.5 m/s in the X direction.
        - No movement in Y/Z directions or yaw.
        """  
        return 0.5, 0.0, 0.0, 0.0  # (vx, vy, vz, yaw_rate)

    def compute_waypoint(self, perception_output):
        """
        Compute waypoint commands based on perception output.
        - Sets a waypoint 5 meters ahead of the current position.
        """
        target_x, target_y, target_z = perception_output["position_offset"]
        return target_x, target_y, target_z, 0.0  # (px, py, pz, yaw)
