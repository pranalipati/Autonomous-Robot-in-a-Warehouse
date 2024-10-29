#!/usr/bin/env python
# coding: utf-8

# In[14]:


import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from queue import PriorityQueue

# Define Warehouse dimensions and Robot attributes
warehouse_width, warehouse_height = 10, 10
start_position = (0, 0)
destination_position = (7, 9)
robot_speed = 0.1  # Robot's speed (m/s)
stop_duration = 2  # Robot's pause duration after each movement (s)

# Define the Warehouse class
class Warehouse:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = set()  # Optional: Define obstacles as needed

    def add_obstacle(self, x, y):
        """Add an obstacle at position (x, y)"""
        self.obstacles.add((x, y))

    def is_within_bounds(self, position):
        """Check if the position is within warehouse bounds"""
        x, y = position
        return 0 <= x < self.width and 0 <= y < self.height

    def is_obstacle(self, position):
        """Check if a position contains an obstacle"""
        return position in self.obstacles

# Define the Robot class with pathfinding using A* algorithm
class Robot:
    def __init__(self, start, destination, warehouse):
        self.position = start
        self.destination = destination
        self.path = []
        self.warehouse = warehouse

    def heuristic(self, a, b):
        """Heuristic function for A* (Manhattan distance)"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_star_search(self):
        """A* search algorithm to find optimal path"""
        open_set = PriorityQueue()
        open_set.put((0, self.position))
        came_from = {}
        g_score = {self.position: 0}
        f_score = {self.position: self.heuristic(self.position, self.destination)}
        
        while not open_set.empty():
            _, current = open_set.get()
            if current == self.destination:
                return self.reconstruct_path(came_from)
            
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self.warehouse.is_within_bounds(neighbor) or self.warehouse.is_obstacle(neighbor):
                    continue

                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, self.destination)
                    open_set.put((f_score[neighbor], neighbor))
        return []

    def reconstruct_path(self, came_from):
        """Reconstruct the path from the destination to start"""
        current = self.destination
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(self.position)
        return path[::-1]

# Initialize Warehouse and Robot
warehouse = Warehouse(warehouse_width, warehouse_height)
robot = Robot(start_position, destination_position, warehouse)
robot.path = robot.a_star_search()  # Generate path using A*

# Set up the plot for animation
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-0.5, warehouse.width - 0.5)
ax.set_ylim(-0.5, warehouse.height - 0.5)
ax.set_xticks(range(warehouse.width))
ax.set_yticks(range(warehouse.height))
ax.grid(True)  # Show grid

# Plot start and destination points
ax.plot(start_position[0], start_position[1], "go", label="Start")  # Green circle for start
ax.plot(destination_position[0], destination_position[1], "ro", label="Destination")  # Red circle for destination

# Plot obstacles (if any)
for obstacle in warehouse.obstacles:
    ax.plot(obstacle[0], obstacle[1], "ks", markersize=12)  # Black square for obstacles

# Initialize robot's position on plot
robot_marker, = ax.plot([], [], "bo", markersize=10)  # Blue circle for robot's position
path_line, = ax.plot([], [], "b--", linewidth=1, alpha=0.5)  # Dashed line to indicate path

# Animation function for updating robot's position
def update(frame):
    if frame < len(robot.path):
        x, y = robot.path[frame]
        robot_marker.set_data(x, y)
        path_x, path_y = zip(*robot.path[:frame+1])
        path_line.set_data(path_x, path_y)
    return robot_marker, path_line

# Create and run the animation
ani = animation.FuncAnimation(fig, update, frames=len(robot.path), interval=300, blit=True)

# Display the plot with a legend
plt.legend()
plt.show()


# In[ ]:


This solution will:

Use matplotlib.animation for smoother animations.
Show the grid representing the warehouse layout.
Highlight the robot’s path dynamically.
Allow optional obstacles in the warehouse.

Warehouse Class: Handles warehouse dimensions, boundary checking, and obstacle management.
Robot Class with A Pathfinding*: Implements the A* algorithm to calculate an efficient path, handling any obstacles that may be present in the warehouse.

Visualization:Grid Display: Shows a 10x10 grid with defined boundaries.
Start and Destination Points: Start is marked in green, and the destination is marked in red.

Obstacles: Black squares represent any obstacles (if added).
Path and Robot Marker: The robot’s position is shown with a blue circle that animates along the path to the destination.

Animation: The FuncAnimation function in matplotlib.animation animates the robot's movement along the calculated path. The update function changes the robot’s position on each frame, dynamically plotting the path as the robot advances.


# In[15]:


import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Define the path as a sequence of (x, y) coordinates (example path)
path = [(0, 0), (1, 1), (2, 2), (3, 2), (4, 3), (5, 4), (6, 6), (7, 7), (7, 8), (7, 9)]

# Prepare 3D plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("3D Path Visualization of Autonomous Robot")

# Set axis limits
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.set_zlim(0, len(path) * 0.1)

# Labels
ax.set_xlabel("X Position")
ax.set_ylabel("Y Position")
ax.set_zlabel("Time (s)")

# Plot start and destination points
start, destination = path[0], path[-1]
ax.scatter(start[0], start[1], 0, color="green", s=100, label="Start")      # Green for start
ax.scatter(destination[0], destination[1], len(path) * 0.1, color="red", s=100, label="Destination")  # Red for destination

# Initialize robot position as a scatter plot in 3D
robot_marker, = ax.plot([], [], [], 'bo', markersize=10)  # Blue circle for robot

# 3D path (x, y, time)
x_vals = [p[0] for p in path]
y_vals = [p[1] for p in path]
z_vals = [i * 0.1 for i in range(len(path))]

# Animation update function
def update(num):
    # Update robot marker's position
    robot_marker.set_data(x_vals[:num+1], y_vals[:num+1])
    robot_marker.set_3d_properties(z_vals[:num+1])
    return robot_marker,

# Run the animation
ani = animation.FuncAnimation(fig, update, frames=len(path), interval=300, blit=True)

# Display plot with legend
plt.legend()
plt.show()


# In[ ]:


3D Visualization with Matplotlib
A 3D view can add depth to the visualization by plotting the robot’s x and y position on a 2D plane while adding time as the z-axis, showing how the robot progresses over time.

3D Plot Setup: The robot’s path is plotted in 3D space, with time represented on the z-axis.
Dynamic Path: As the animation proceeds, the robot marker’s position updates, creating a 3D trajectory from start to destination.
Axis Labels: X and Y represent spatial coordinates, while Z (time axis) allows you to visualize movement over time.


# In[16]:


import plotly.graph_objects as go
import numpy as np

# Define a sample path for demonstration (x, y coordinates)
path = [(0, 0), (1, 1), (2, 2), (3, 2), (4, 3), (5, 4), (6, 6), (7, 7), (7, 8), (7, 9)]

# Unpack the path coordinates
x_vals, y_vals = zip(*path)
time_vals = list(range(len(path)))

# Initialize a 2D scatter plot with Plotly
fig = go.Figure()

# Plot path as a line
fig.add_trace(go.Scatter(x=x_vals, y=y_vals, mode="lines+markers", name="Path"))

# Define start and destination points with custom markers
fig.add_trace(go.Scatter(x=[x_vals[0]], y=[y_vals[0]], mode="markers", marker=dict(color="green", size=10), name="Start"))
fig.add_trace(go.Scatter(x=[x_vals[-1]], y=[y_vals[-1]], mode="markers", marker=dict(color="red", size=10), name="Destination"))

# Update frames for the robot movement
frames = [go.Frame(data=[go.Scatter(x=[x_vals[k]], y=[y_vals[k]], mode="markers", marker=dict(color="blue", size=12))],
                   name=f'Frame{k}') for k in range(len(path))]

fig.frames = frames

# Set up animation
fig.update_layout(
    title="Interactive Autonomous Robot Simulation",
    xaxis=dict(range=[-1, 10], title="X Position"),
    yaxis=dict(range=[-1, 10], title="Y Position"),
    updatemenus=[dict(type="buttons",
                      showactive=False,
                      buttons=[dict(label="Play",
                                    method="animate",
                                    args=[None, {"frame": {"duration": 300, "redraw": True}, "fromcurrent": True}]),
                               dict(label="Pause",
                                    method="animate",
                                    args=[[None], {"frame": {"duration": 0, "redraw": False}, "mode": "immediate",
                                                   "fromcurrent": True}])])]
)

fig.show()


# In[ ]:


Interactive Visualization with Plotly
Using Plotly enables you to create interactive animations that can be explored in a web browser. Plotly is especially effective if you want to highlight specific parts of the path or provide zoom-in capabilities.

Plotly Figure Setup: Defines a figure with a path, start, and destination markers.
Frames for Animation: Each frame represents one step of the robot’s movement along the path. Plotly’s interactive environment allows you to pan, zoom, and explore the path.
Play/Pause Animation: Enables control over animation playback, making it ideal for interactive presentations.

