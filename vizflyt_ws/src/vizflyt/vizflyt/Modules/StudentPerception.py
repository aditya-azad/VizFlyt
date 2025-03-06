class StudentPerception:
    """
    Sample student-implemented perception module.
    Processes RGB and Depth images and extracts relevant information.
    """

    def process_images(self, rgb_image, depth_image):
        """
        Dummy function to process images.
        This function simulates object detection and scene understanding.
        """
        # Example: Extract relevant features from images (placeholder)
        perception_output = {
            "detected_objects": [],  # No objects detected in this placeholder
            "position_offset": (5.0, 0.0, 0.0)  # Assume target is 5 meters ahead
        }
        return perception_output
