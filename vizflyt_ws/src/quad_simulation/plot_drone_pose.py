import numpy as np
import scipy.io
import matplotlib.pyplot as plt

data = scipy.io.loadmat('quad_simulation/log/drone_pose.mat')

positions = data['position']
x, y, z = positions[:, 0], positions[:, 1], positions[:, 2]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(x, y, -z, label='Drone Trajectory', linewidth=2)
ax.scatter(x[0], y[0], -z[0], color='g', marker='o', label='Start')
ax.scatter(x[-1], y[-1], -z[-1], color='r', marker='x', label='End')

ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m] (Altitude)')

ax.set_title('Drone Trajectory Visualization')
ax.legend()

try:
    print("Press Ctrl+C to close the visualization.")
    plt.show()
except KeyboardInterrupt:
    print("\nVisualization closed by user (Ctrl+C).")

