import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Read the interpolated spline points
data = pd.read_csv("circle_spline.csv")

# Extract X and Y coordinates
X_interp = data["X"]
Y_interp = data["Y"]

# Create a figure and axis
fig, ax = plt.subplots(figsize=(8, 8))

# Plot the unit circle
theta = np.linspace(0, np.pi, 200)
x_circle = np.cos(theta)
y_circle = np.sin(theta)
ax.plot(x_circle, y_circle, label="Unit Circle", color="lightgray", linewidth=2)

# Plot the original test points
test_points = pd.read_csv("circle_cones.csv")
ax.scatter(
    test_points["X"], test_points["Y"], color="red", label="Test Points", zorder=5
)

# Plot the interpolated spline points
ax.scatter(X_interp, Y_interp, label="Spline Fit", color="blue", linewidth=2)

# Configure the plot
ax.set_aspect("equal", "box")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_title("Spline Fit vs. Unit Circle")
ax.legend()
ax.grid(True)

# Show the plot
plt.show()
