import numpy as np

# Load CSV file (9 rows, N columns for trajectory points)
mp_ned = np.genfromtxt("quad_simulation/traj/MP.csv", delimiter=',', skip_header=0)

# Convert Position (Rows 0-2), Velocity (Rows 3-5), and Acceleration (Rows 6-8)
mp_nwu = np.copy(mp_ned)  # Create a copy to modify

# 1. Flip Y and Z for position (Rows 0-2)
mp_nwu[1, :] = -mp_ned[1, :]  # Flip Y-axis
mp_nwu[2, :] = -mp_ned[2, :]  # Flip Z-axis

# 2. Flip Y and Z for velocity (Rows 3-5)
mp_nwu[4, :] = -mp_ned[4, :]  # Flip Vy
mp_nwu[5, :] = -mp_ned[5, :]  # Flip Vz

# 3. Flip Y and Z for acceleration (Rows 6-8)
mp_nwu[7, :] = -mp_ned[7, :]  # Flip Ay
mp_nwu[8, :] = -mp_ned[8, :]  # Flip Az

# Save the converted CSV file
np.savetxt("quad_simulation/traj/MP_NWU.csv", mp_nwu, delimiter=',')

print("Converted MP.csv from NED to NWU and saved as MP_NWU.csv")
