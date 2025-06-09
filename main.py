from splat_render import SplatRenderer
import numpy as np
import cv2

parent_dir = "/home/deepak/Deepak/PhD/research/VizFlyt/goddard/goddard_region_2_splat/splatfacto/2025-06-07_160008"
config_path = parent_dir + "/config.yml"
json_path = "/home/deepak/Deepak/PhD/research/VizFlyt/vizflyt_ws/src/vizflyt_viewer/render_settings/render_config.json"

renderer = SplatRenderer(config_path, json_path)

# Position: x, y, z in meters | Orientation: roll, pitch, yaw in radians
pose = np.array([0.0, 0.0, 0.0])            # NED origin
rpy = np.radians([0.0, 0.0, 0.0])           # Level facing forward

rgb, depth = renderer.render(pose, rpy)

print('RGB shape ', rgb.shape)

cv2.imwrite('img.jpg', rgb)
cv2.imwrite('depth.jpg', depth)