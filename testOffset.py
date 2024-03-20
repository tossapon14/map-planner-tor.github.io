import matplotlib.pyplot as plt
import numpy as np
from Mapping.utils import offset_polygon

# Example polygon vertices
# ox = [-5.0, 10, 12,20]
# oy = [0.0, 10,5.0, -5.0]
polygon = np.array([[0, 0], [10, 3], [8,6],[8, 10],[2,10]])
polygonring = np.array([[0, 0], [10, 3], [8,6],[8, 10],[2,10],[0,0]])
# Offset distance (inside)
offset_distance = -1  # Negative for inside offset

# Calculate offset polygon vertices
offset_vertices = offset_polygon(polygon, offset_distance)



# Plot original and offset polygons
plt.plot(polygonring[:, 0], polygonring[:, 1], label='Original Polygon', color='blue')
plt.plot(offset_vertices[:, 0], offset_vertices[:, 1], label='Offset Polygon (Inside)', color='red')

# Plot vertices for reference
plt.scatter(polygon[:, 0], polygon[:, 1], color='blue')
plt.scatter(offset_vertices[:, 0], offset_vertices[:, 1], color='red')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Original and Inside Offset Polygons')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
