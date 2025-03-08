# Copyright 2025 Tudor Oancea, Mateo Berthet
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

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
