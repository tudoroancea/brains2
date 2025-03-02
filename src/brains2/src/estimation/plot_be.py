import matplotlib.pyplot as plt
import pandas as pd
import os
import numpy as np

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

# plot track_width cones[cones["color"] == "track_width"]["X"], cones[cones["color"] == "track_width"]["Y"] in funtion of the index
track_width_left = cones[cones["color"] == "track_width"]["X"]
track_width_right = cones[cones["color"] == "track_width"]["Y"]
plt.plot(
    np.linspace(0, len(track_width_left) - 1, len(track_width_left)),
    track_width_left,
    label="track_width X",
)
plt.plot(
    np.linspace(0, len(track_width_right) - 1, len(track_width_right)),
    track_width_right,
    label="track_width Y",
)

plt.legend()
plt.show()
