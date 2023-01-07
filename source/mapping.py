import matplotlib.pyplot as plt
import numpy as np

from blocks import Map, Car
from constants import ORIGIN, TOP_LEFT_CORNER, TOP_RIGHT_CORNER, BOTTOM_LEFT_CORNER, BOTTOM_RIGHT_CORNER



# Load the image file and convert it to a NumPy array
image = plt.imread('images/map_improved.png')

destination = np.array([38.736911, -9.139010])

# Get the map
map = Map()
coords = map.get_path(ORIGIN, destination)

top_right_corner = map.get_coordinates(TOP_RIGHT_CORNER[0], TOP_RIGHT_CORNER[1])
bottom_left_corner = map.get_coordinates(BOTTOM_LEFT_CORNER[0], BOTTOM_LEFT_CORNER[1])
bottom_right_corner = map.get_coordinates(BOTTOM_RIGHT_CORNER[0], BOTTOM_RIGHT_CORNER[1])
top_left_corner = map.get_coordinates(TOP_LEFT_CORNER[0], TOP_LEFT_CORNER[1])

# Get the coordinates of the origin point
origin = map.get_coordinates(ORIGIN[0], ORIGIN[1])

# For debugging
print(f"{round(top_left_corner[0][0])}, {round(top_left_corner[0][1])} ------ {round(top_right_corner[0][0])}, {round(top_right_corner[0][1])}")
print(f"{round(bottom_left_corner[0][0])}, {round(bottom_left_corner[0][1])} ------ {round(bottom_right_corner[0][0])}, {round(bottom_right_corner[0][1])}")

# Plot the point on the map
fig, ax = plt.subplots()  # Create a figure and axes object

# Set the x-axis and y-axis limits to center the origin point
x_lims = [round(bottom_left_corner[0][0]) - round(origin[0][0]), round(top_right_corner[0][0]) - round(origin[0][0])]
y_lims = [round(bottom_left_corner[0][1]) - round(origin[0][1]), round(top_right_corner[0][1]) - round(origin[0][1])]


ax.set_xlim(x_lims[0], x_lims[1])
ax.set_ylim(y_lims[0], y_lims[1])
ax.imshow(image, extent=[x_lims[0], x_lims[1], y_lims[0], y_lims[1]])  # Plot the image on the axes

for i in range(len(coords)):
    x = coords[i][0] - round(origin[0][0])
    y = coords[i][1] - round(origin[0][1])
    ax.plot(x, y, 'ro', markersize=3)  # Plot the path on the axes

# Show the plot
plt.show()

