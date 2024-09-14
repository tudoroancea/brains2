# Copyright (c) 2024. Tudor Oancea, Matteo Berthet
std_msgs/Header header
float64[] s_cen  # arc lengths of the points on the center line, m
float64[] x_cen  # X positions of the points, m
float64[] y_cen  # Y positions of the points, m
float64[] phi_cen  # yaw of the tangets to the points, rad
float64[] kappa_cen  # curvature at the points, 1/m
float64[] w_cen  # half track width at the points (between the center line and the boundaries), m
