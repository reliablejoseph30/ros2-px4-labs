import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('/home/toba/mission_log_20260320_152919.csv')

# Plot 1: Altitude over time
plt.figure(figsize=(12,5))
plt.plot(df['ros_time'], df['z'], color='blue')
plt.xlabel('ROS Time (s)')
plt.ylabel('Altitude Z (m)')
plt.title('Altitude vs Time — Takeoff, Waypoints, RTL')
plt.grid(True)
plt.tight_layout()
plt.savefig('/home/toba/plot_altitude.png', dpi=150)
plt.show()

# Plot 2: Ground track
plt.figure(figsize=(8,8))
plt.plot(df['x'], df['y'], marker='.', markersize=2, color='green')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Ground Track — Lawnmower Pattern')
plt.axis('equal')
plt.grid(True)
plt.tight_layout()
plt.savefig('/home/toba/plot_groundtrack.png', dpi=150)
plt.show()

print("Plots saved to home directory!")
