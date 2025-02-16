# the method to isolate the center line is inspired by the following blog post
# https://blogs.mathworks.com/student-lounge/2022/10/03/path-planning-for-formula-student-driverless-cars-using-delaunay-triangulation/

import numpy as np
import scipy
import matplotlib.pyplot as plt
from scipy.sparse import csc_array
from scipy.sparse import eye as speye
from scipy.sparse import kron as spkron
from qpsolvers import solve_qp
from icecream import ic

track_name = "gamma"
data = np.loadtxt(f"{track_name}.csv", delimiter=",", dtype=str, skiprows=1)
# extract numpy arrays
blue_cones = data[data[:, 0] == "blue", 1:].astype(float)
yellow_cones = data[data[:, 0] == "yellow", 1:].astype(float)
orange_cones = data[data[:, 0] == "orange", 1:].astype(float)
# blue_cones = blue_cones[:10]
# yellow_cones = yellow_cones[:10]
all_cones = np.vstack((blue_cones, yellow_cones))

# distance matrix between cones
dist_matrix = scipy.spatial.distance.cdist(all_cones, all_cones)

# create delaunay triangulation
tri = scipy.spatial.Delaunay(all_cones)

# find exterior triangles
simpl = tri.simplices
is_yellow_cone_mask = simpl >= len(blue_cones)
tpr = np.sum(is_yellow_cone_mask.astype(int), 1)
exterior_triangle_mask = np.logical_or(tpr == 3, tpr == 0)
interior_triangle_mask = np.logical_not(exterior_triangle_mask)

# in all the interior triangles, extract the edges that cross the track
crossing_edges = set()
for triangle in simpl[interior_triangle_mask]:
    for edge in [
        (triangle[0], triangle[1]),
        (triangle[1], triangle[2]),
        (triangle[2], triangle[0]),
    ]:
        if (edge[0] < len(blue_cones)) != (edge[1] < len(blue_cones)):
            crossing_edges.add(tuple(sorted(edge)))

middle_points = [
    0.5 * (all_cones[edge[0]] + all_cones[edge[1]]) for edge in crossing_edges
]

# order middle points to form a continuous line
new_middle_points = [np.array([0, 0])]
while len(middle_points) > 0:
    min_idx = np.argmin(np.linalg.norm(middle_points - new_middle_points[-1], axis=1))
    new_middle_points.append(middle_points[min_idx])
    middle_points = np.delete(middle_points, min_idx, axis=0)
middle_points = np.array(new_middle_points)

ic(middle_points[0], middle_points[-1])


def fit_spline(
    path: np.ndarray,
    curv_weight: float = 1.0,
) -> tuple[np.ndarray, np.ndarray]:
    """
    computes the coefficients of each spline portion of the path.
    > Note: the path is assumed to be closed but the first and last points are NOT the same.

    :param path: Nx2 array of points
    :param curv_weight: weight of the curvature term in the cost function
    :return_errs:
    :qp_solver:
    :returns p_X, p_Y: Nx4 arrays of coefficients of the splines in the x and y directions
                       (each row correspond to a_i, b_i, c_i, d_i coefficients of the i-th
                       spline portion defined by a_i + b_i * t + ...)
    """
    assert len(path.shape) == 2 and path.shape[1] == 2, (
        f"path must have shape (N,2) but has shape {path.shape}"
    )

    # precompute all the QP data
    N = path.shape[0]
    delta_s = np.linalg.norm(path[1:] - path[:-1], axis=1)
    delta_s = np.append(delta_s, np.linalg.norm(path[0] - path[-1]))
    rho = np.zeros(N)
    rho[:-1] = delta_s[:-1] / delta_s[1:]
    rho[-1] = delta_s[-1] / delta_s[0]
    IN = speye(N, format="csc")
    A = spkron(
        IN,
        np.array(
            [
                [1.0, 1.0, 1.0, 1.0],
                [0.0, 1.0, 2.0, 3.0],
                [0.0, 0.0, 2.0, 6.0],
            ]
        ),
        format="csc",
    ) + csc_array(
        (
            np.concatenate((-np.ones(N), -rho, -2 * rho**2)),
            (
                np.concatenate(
                    (
                        3 * np.arange(N),
                        1 + 3 * np.arange(N),
                        2 + 3 * np.arange(N),
                    )
                ),
                np.concatenate(
                    (
                        np.roll(4 * np.arange(N), -1),
                        np.roll(1 + 4 * np.arange(N), -1),
                        np.roll(2 + 4 * np.arange(N), -1),
                    )
                ),
            ),
        ),
        shape=(3 * N, 4 * N),
    )
    B = spkron(IN, np.array([[1.0, 0.0, 0.0, 0.0]]), format="csc")
    C = csc_array(
        (
            np.concatenate((2 / np.square(delta_s), 6 / np.square(delta_s))),
            (
                np.concatenate((np.arange(N), np.arange(N))),
                np.concatenate((2 + 4 * np.arange(N), 3 + 4 * np.arange(N))),
            ),
        ),
        shape=(N, 4 * N),
    )
    P = B.T @ B + curv_weight * C.T @ C + 1e-10 * speye(4 * N, format="csc")
    q = -B.T @ path
    b = np.zeros(3 * N)

    # solve the QP for X and Y separately
    p_X = solve_qp(P=P, q=q[:, 0], A=A, b=b, solver="osqp")
    p_Y = solve_qp(P=P, q=q[:, 1], A=A, b=b, solver="osqp")
    if p_X is None or p_Y is None:
        raise ValueError("solving qp failed")

    # compute interpolation error on X and Y
    # X_err = B @ p_X - path[:, 0]
    # Y_err = B @ p_Y - path[:, 1]

    # reshape to (N,4) arrays
    p_X = np.reshape(p_X, (N, 4))
    p_Y = np.reshape(p_Y, (N, 4))

    return p_X, p_Y


def check_spline_coeffs_dims(coeffs_X: np.ndarray, coeffs_Y: np.ndarray):
    assert len(coeffs_X.shape) == 2 and coeffs_X.shape[1] == 4, (
        f"coeffs_X must have shape (N,4) but has shape {coeffs_X.shape}"
    )
    assert len(coeffs_Y.shape) == 2 and coeffs_Y.shape[1] == 4, (
        f"coeffs_Y must have shape (N,4) but has shape {coeffs_Y.shape}"
    )
    assert coeffs_X.shape[0] == coeffs_Y.shape[0], (
        f"coeffs_X and coeffs_Y must have the same length but have lengths {coeffs_X.shape[0]} and {coeffs_Y.shape[0]}"
    )


def compute_spline_interval_lengths(
    coeffs_X: np.ndarray, coeffs_Y: np.ndarray, no_interp_points=100
):
    """
    computes the lengths of each spline portion of the path.
    > Note: Here the closeness of the part does not matter, it is contained in the coefficients

    :param coeff_X: Nx4 array of coefficients of the splines in the x direction (as returned by calc_splines)
    :param coeff_Y: Nx4 array of coefficients of the splines in the y direction (as returned by calc_splines)
    :param delta_s: number of points to use on each spline portion for the interpolation
    """
    check_spline_coeffs_dims(coeffs_X, coeffs_Y)

    N = coeffs_X.shape[0]

    t_steps = np.linspace(0.0, 1.0, no_interp_points)[np.newaxis, :]
    interp_points = np.zeros((no_interp_points, N, 2))

    interp_points[:, :, 0] = coeffs_X[:, 0]
    interp_points[:, :, 1] = coeffs_Y[:, 0]

    coeffs_X = coeffs_X[:, np.newaxis, :]
    coeffs_Y = coeffs_Y[:, np.newaxis, :]

    interp_points = interp_points.transpose(1, 0, 2)

    interp_points[:, :, 0] += coeffs_X[:, :, 1] @ t_steps
    interp_points[:, :, 0] += coeffs_X[:, :, 2] @ np.power(t_steps, 2)
    interp_points[:, :, 0] += coeffs_X[:, :, 3] @ np.power(t_steps, 3)

    interp_points[:, :, 1] += coeffs_Y[:, :, 1] @ t_steps
    interp_points[:, :, 1] += coeffs_Y[:, :, 2] @ np.power(t_steps, 2)
    interp_points[:, :, 1] += coeffs_Y[:, :, 3] @ np.power(t_steps, 3)

    delta_s = np.sum(
        np.sqrt(np.sum(np.power(np.diff(interp_points, axis=1), 2), axis=2)), axis=1
    )
    assert delta_s.shape == (N,), f"{delta_s.shape}"
    return delta_s


def uniformly_sample_spline(
    coeffs_X: np.ndarray,
    coeffs_Y: np.ndarray,
    delta_s: np.ndarray,
    n_samples: int,
):
    """
    uniformly n_samples equidistant points along the path defined by the splines.
    The first point will always be the initial point of the first spline portion, and
    the last point will NOT be the initial point of the first spline portion.

    :param coeffs_X: Nx4 array of coefficients of the splines in the x direction (as returned by calc_splines)
    :param coeffs_Y: Nx4 array of coefficients of the splines in the y direction (as returned by calc_splines)
    :param spline_lengths: N array of lengths of the spline portions (as returned by calc_spline_lengths)
    :param n_samples: number of points to sample

    :return X_interp: n_samples array of X coordinates along the path
    :return Y_interp: n_samples array of Y coordinates along the path
    :return idx_interp: n_samples array of indices of the spline portions that host the points
    :return t_interp: n_samples array of t values of the points within their respective spline portions
    :return s_interp: n_samples array of distances along the path of the points
    """
    s = np.cumsum(delta_s)
    s_interp = np.linspace(0.0, s[-1], n_samples, endpoint=False)

    # find the spline that hosts the current interpolation point
    idx_interp = np.argmax(s_interp[:, np.newaxis] < s, axis=1)

    t_interp = np.zeros(n_samples)  # save t values
    X_interp = np.zeros(n_samples)  # raceline coords
    Y_interp = np.zeros(n_samples)  # raceline coords

    # get spline t value depending on the progress within the current element
    t_interp[idx_interp > 0] = (
        s_interp[idx_interp > 0] - s[idx_interp - 1][idx_interp > 0]
    ) / delta_s[idx_interp][idx_interp > 0]
    t_interp[idx_interp == 0] = s_interp[idx_interp == 0] / delta_s[0]

    # calculate coords
    X_interp = (
        coeffs_X[idx_interp, 0]
        + coeffs_X[idx_interp, 1] * t_interp
        + coeffs_X[idx_interp, 2] * np.power(t_interp, 2)
        + coeffs_X[idx_interp, 3] * np.power(t_interp, 3)
    )

    Y_interp = (
        coeffs_Y[idx_interp, 0]
        + coeffs_Y[idx_interp, 1] * t_interp
        + coeffs_Y[idx_interp, 2] * np.power(t_interp, 2)
        + coeffs_Y[idx_interp, 3] * np.power(t_interp, 3)
    )

    return X_interp, Y_interp, idx_interp, t_interp, s_interp


def get_heading(
    coeffs_X: np.ndarray,
    coeffs_Y: np.ndarray,
    idx_interp: np.ndarray,
    t_interp: np.ndarray,
) -> np.ndarray:
    """
    analytically computes the heading and the curvature at each point along the path
    specified by idx_interp and t_interp.

    :param coeffs_X: Nx4 array of coefficients of the splines in the x direction (as returned by calc_splines)
    :param coeffs_Y: Nx4 array of coefficients of the splines in the y direction (as returned by calc_splines)
    :param idx_interp: n_samples array of indices of the spline portions that host the points
    :param t_interp: n_samples array of t values of the points within their respective spline portions
    """
    check_spline_coeffs_dims(coeffs_X, coeffs_Y)

    # we don't divide by delta_s[idx_interp] here because this term will cancel out
    # in arctan2 either way
    x_d = (
        coeffs_X[idx_interp, 1]
        + 2 * coeffs_X[idx_interp, 2] * t_interp
        + 3 * coeffs_X[idx_interp, 3] * np.square(t_interp)
    )
    y_d = (
        coeffs_Y[idx_interp, 1]
        + 2 * coeffs_Y[idx_interp, 2] * t_interp
        + 3 * coeffs_Y[idx_interp, 3] * np.square(t_interp)
    )
    phi = np.arctan2(y_d, x_d)

    return phi


def get_curvature(
    coeffs_X: np.ndarray,
    coeffs_Y: np.ndarray,
    idx_interp: np.ndarray,
    t_interp: np.ndarray,
) -> np.ndarray:
    # same here with the division by delta_s[idx_interp] ** 2
    x_d = (
        coeffs_X[idx_interp, 1]
        + 2 * coeffs_X[idx_interp, 2] * t_interp
        + 3 * coeffs_X[idx_interp, 3] * np.square(t_interp)
    )
    y_d = (
        coeffs_Y[idx_interp, 1]
        + 2 * coeffs_Y[idx_interp, 2] * t_interp
        + 3 * coeffs_Y[idx_interp, 3] * np.square(t_interp)
    )
    x_dd = 2 * coeffs_X[idx_interp, 2] + 6 * coeffs_X[idx_interp, 3] * t_interp
    y_dd = 2 * coeffs_Y[idx_interp, 2] + 6 * coeffs_Y[idx_interp, 3] * t_interp
    kappa = (x_d * y_dd - y_d * x_dd) / np.power(x_d**2 + y_d**2, 1.5)
    return kappa


def unwrap_to_pi(x: np.ndarray) -> np.ndarray:
    """remove discontinuities caused by wrapToPi"""
    diffs = np.diff(x)
    diffs[diffs > 1.5 * np.pi] -= 2 * np.pi
    diffs[diffs < -1.5 * np.pi] += 2 * np.pi
    return np.insert(x[0] + np.cumsum(diffs), 0, x[0])


# splines and stuff
coeffs_X, coeffs_Y = fit_spline(path=middle_points, curv_weight=2.0)
delta_s = compute_spline_interval_lengths(coeffs_X=coeffs_X, coeffs_Y=coeffs_Y)
X_cen, Y_cen, idx_interp, t_interp, s_cen = uniformly_sample_spline(
    coeffs_X=coeffs_X, coeffs_Y=coeffs_Y, delta_s=delta_s, n_samples=500
)
phi_cen = get_heading(coeffs_X, coeffs_Y, idx_interp, t_interp)
kappa_cen = get_curvature(coeffs_X, coeffs_Y, idx_interp, t_interp)

lap_length = s_cen[-1] + np.hypot(X_cen[-1] - X_cen[0], Y_cen[-1] - Y_cen[0])
s_cen = np.concatenate((s_cen - lap_length, s_cen, s_cen + lap_length))
X_cen = np.concatenate((X_cen, X_cen, X_cen))
Y_cen = np.concatenate((Y_cen, Y_cen, Y_cen))
phi_cen = unwrap_to_pi(np.concatenate((phi_cen, phi_cen, phi_cen)))
kappa_cen = np.concatenate((kappa_cen, kappa_cen, kappa_cen))
np.savetxt(
    f"{track_name}_center_line.txt",
    np.column_stack(
        (s_cen, X_cen, Y_cen, phi_cen, kappa_cen, 1.5 * np.ones_like(s_cen))
    ),
    header="s,X,Y,phi,kappa,w",
    comments="",
)

# plot things
plt.triplot(
    all_cones[:, 0],
    all_cones[:, 1],
    simpl[~exterior_triangle_mask],
    color="black",
    linewidth=1,
)
plt.triplot(
    all_cones[:, 0],
    all_cones[:, 1],
    simpl[exterior_triangle_mask],
    color="red",
    linewidth=0.5,
)
plt.scatter(blue_cones[:, 0], blue_cones[:, 1], color="blue", marker="^", s=10)
plt.scatter(yellow_cones[:, 0], yellow_cones[:, 1], color="yellow", marker="^", s=10)
plt.scatter(orange_cones[:, 0], orange_cones[:, 1], color="orange", marker="^", s=20)
plt.plot(middle_points[:, 0], middle_points[:, 1], color="green", linewidth=2)
plt.axis("equal")
plt.tight_layout()
# plt.savefig("BE.png", dpi=300)
plt.show()
