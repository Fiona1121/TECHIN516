import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import pandas as pd

def update_lines(num, data, line):
    """Update function for animation"""
    line.set_data(data[0:2, :num])    
    line.set_3d_properties(data[2, :num])    
    return line

# Create figure and 3D axis
fig = plt.figure()
ax = p3.Axes3D(fig, auto_add_to_figure=False)
fig.add_axes(ax)

# Read CSV data (excluding time column)
repo = pd.read_csv('data.csv', sep=',', header=0, usecols=["x", "y", "z"])
data = np.array((repo['x'].values, repo['y'].values, repo['z'].values))

print("Number of data points: ", data.shape[1])

# Plot the Bezier trajectory (actual execution)
line = ax.plot(data[0, 0:1], data[1, 0:1], data[2, 0:1], color='blue', label="Bezier Trajectory")[0]

# Bezier control points
bezier_points = np.array([
    [0.576, 0.407, 0.299, 0.388],  # x
    [0.002, 0.121, 0.466, 0.598],  # y
    [0.434, 0.667, 0.740, 0.535]   # z
])

# Plot control points in red
ax.scatter(bezier_points[0], bezier_points[1], bezier_points[2], color='red', marker='o', s=80, label="Bezier Control Points")

# Add labels for start and end positions
ax.text(bezier_points[0][0], bezier_points[1][0], bezier_points[2][0], "Start", color='black')
ax.text(bezier_points[0][-1], bezier_points[1][-1], bezier_points[2][-1], "End", color='black')

# Automatically set axis limits based on data
ax.set_xlim3d([np.min(data[0])-0.1, np.max(data[0])+0.1])
ax.set_ylim3d([np.min(data[1])-0.1, np.max(data[1])+0.1])
ax.set_zlim3d([np.min(data[2])-0.1, np.max(data[2])+0.1])

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Improved 3D Position Visualization')
ax.legend()

# Create the Animation object
line_ani = animation.FuncAnimation(fig, update_lines, data.shape[1], fargs=(data, line), interval=200, blit=False)

plt.show()
