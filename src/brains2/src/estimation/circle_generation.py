# generate_test_points.py
import numpy as np
import pandas as pd

# Number of test points to generate
num_test_points = 6  # Adjust this number as desired (e.g., 50, 100, 200)

# Generate angles between 0 and pi (half-circle)
theta = np.linspace(0, np.pi, num_test_points)

# Compute X and Y coordinates on the unit circle
X = np.cos(theta) * 2.5
Y = np.sin(theta) * 2.5

# Create a DataFrame
df = pd.DataFrame({"color": "blue", "X": X, "Y": Y})

# Write to CSV
df.to_csv("circle_cones.csv", index=False)
