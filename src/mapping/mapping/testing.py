import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Step 1: Load the data from the .json file
file_path = '/home/kristoffer-germalm/dd2419_ws/map_file.json'  # Replace with the actual path to your JSON file

# Open the file and load the JSON data
with open(file_path, 'r') as f:
    json_data = json.load(f)

print(f"Number of items: {len(json_data)}")

# Step 2: Prepare the data
x_vals = []
y_vals = []
z_vals = []
labels = []

# Loop through the JSON data and extract the values
for key, value in json_data.items():
    x_vals.append(value['x'])
    y_vals.append(value['y'])
    z_vals.append(value['z'])
    labels.append(value['label'])

# Step 3: Create the 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_zlim(0.5, 2)  # Example: Setting the Z-axis range from 0 to 100

# Plot the points in 3D space
ax.scatter(x_vals, y_vals, z_vals, c='r', marker='o')

# Add labels for each point
for i, label in enumerate(labels):
    ax.text(x_vals[i], y_vals[i], z_vals[i], label, color='black')

# Set labels for axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Show the plot
plt.show()
