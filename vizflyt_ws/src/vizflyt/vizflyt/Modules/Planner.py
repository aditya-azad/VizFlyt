class Planner:
    """
    Planner class that integrates student-implemented Perception and Motion Planning logic.
    Generates either velocity or position commands based on mode.
    """
    def __init__(self, mode="velocity", perception_module=None, motion_planning_module=None):
        """
        Initializes the Planner.
        :param mode: "velocity" or "position"
        :param perception_module: Student-implemented Perception logic
        :param motion_planning_module: Student-implemented Motion Planning logic
        """
        if mode not in ["velocity", "position"]:
            raise ValueError("Invalid mode. Choose 'velocity' or 'position'.")
        
        self.mode = mode
        self.perception_module = perception_module
        self.motion_planning_module = motion_planning_module

    def compute_command(self, rgb_image, depth_image):
        """
        Calls student-implemented perception and motion planning logic to generate commands.
        """
        # Step 1: Process Images with Perception Module (Student's Code)
        perception_output = self.perception_module.process_images(rgb_image, depth_image)

        # Step 2: Use Motion Planning Module to Generate Commands
        if self.mode == "velocity":
            return self.motion_planning_module.compute_velocity(perception_output)
        else:
            return self.motion_planning_module.compute_waypoint(perception_output)
