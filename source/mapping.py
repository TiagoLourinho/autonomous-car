import matplotlib.pyplot as plt
import numpy as np

from blocks import Map, Car
from constants import ORIGIN

SCALE = 11

# Load the image file and convert it to a NumPy array
image = plt.imread('images/map.png')

top_coord_image = np.array([38.737928, -9.141506])

destination = np.array([38.737967, -9.138895])

# Get the map
map = Map()
coords = SCALE * map.get_path(ORIGIN, destination)
coords[:, [0, 1]] = -coords[:, [1, 0]]
top_coord_image = SCALE * map.get_coordinates(top_coord_image[0], top_coord_image[1], ORIGIN)
top_coord_image[0, [0, 1]] = -top_coord_image[0, [1, 0]]
top_coord_image[0][0] -= 500
top_coord_image[0][1] += 750

print(coords)
print(top_coord_image)

# Plot the point on the map
fig, ax = plt.subplots()  # Create a figure and axes object

# Set the x-axis and y-axis limits to include the origin point
image_width = image.shape[1]  # Get the width of the image in pixels
image_height = image.shape[0]  # Get the height of the image in pixels
x_lims = [top_coord_image[0][0], top_coord_image[0][0] + image_width]
y_lims = [-image_height + top_coord_image[0][1], top_coord_image[0][1]]


ax.set_xlim(x_lims[0], x_lims[1])
ax.set_ylim(y_lims[0], y_lims[1])
ax.imshow(image, extent=[x_lims[0], x_lims[1], y_lims[0], y_lims[1]])  # Plot the image on the axes

for i in range(len(coords)):
    x = coords[i][0]
    y = coords[i][1]
    ax.plot(x, y, 'ro', markersize=3)  # Plot the path on the axes

# Show the plot
plt.show()

