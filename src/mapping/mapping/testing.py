import json
import numpy as np
import matplotlib.pyplot as plt

# Load stored map file
with open("map_file.json", "r") as f:
    object_map = json.load(f)

# Convert to numpy array
points = np.array(list(object_map.values()))

# Scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=10, c="blue", label="Detected Objects")

# Labels and settings
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Stored Detected Objects")
ax.legend()
plt.show()
