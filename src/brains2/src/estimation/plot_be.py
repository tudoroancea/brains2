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

import matplotlib.pyplot as plt
import pandas as pd
import os

# color,X,Y for blue and yellow cones with x and y the coordinates in meters
# cones = pd.read_csv(os.path.dirname(__file__) + "/interpolated_spline.csv")
cones = pd.read_csv(os.path.dirname(__file__) + "/interpolated_spline_alpha.csv")
# gt = pd.read_csv(os.path.dirname(__file__) + "/test_cones.csv")
gt = pd.read_csv(os.path.dirname(__file__) + "/alpha_cones.csv")

# plot a scatter plot of the cones with a bigger size for the ground truth cones
plt.scatter(
    cones[cones["color"] == "blue"]["X"],
    cones[cones["color"] == "blue"]["Y"],
    c="blue",
    label="blue cone",
)
plt.scatter(
    cones[cones["color"] == "yellow"]["X"],
    cones[cones["color"] == "yellow"]["Y"],
    c="yellow",
    label="yellow cone",
)
plt.scatter(
    gt[gt["color"] == "blue"]["X"],
    gt[gt["color"] == "blue"]["Y"],
    c=(0, 0, 0.5),  # RGB for dark blue
    label="blue cone",
    s=100,
)
plt.scatter(
    gt[gt["color"] == "yellow"]["X"],
    gt[gt["color"] == "yellow"]["Y"],
    c=(0.8, 0.8, 0),  # RGB for dark yellow
    label="yellow cone",
    s=100,
)

plt.scatter(
    cones[cones["color"] == "center"]["X"],
    cones[cones["color"] == "center"]["Y"],
    c="red",
    label="alpha cone",
)

plt.legend()
# set the aspect ratio to be equal
plt.axis("equal")
plt.tight_layout()

# find the min and max of the x and y coordinates
min_x = min(cones["X"].min(), gt["X"].min())
max_x = max(cones["X"].max(), gt["X"].max())
min_y = min(cones["Y"].min(), gt["Y"].min())
max_y = max(cones["Y"].max(), gt["Y"].max())
# set the limits of the plot
plt.xlim(min_x - 5, max_x + 5)
plt.ylim(min_y - 5, max_y + 5)
plt.show()
