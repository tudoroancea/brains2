import numpy as np
import numpy.typing as npt
from qpsolvers import available_solvers, solve_qp
from scipy.sparse import csc_array, hstack, vstack
from scipy.sparse import eye as speye
from scipy.sparse import kron as spkron

assert "osqp" in available_solvers

FloatArray = npt.NDArray[np.float64]
IntArray = npt.NDArray[np.int64]

NUMBER_SPLINE_INTERVALS = 500

__all__ = [
    "fit_closed_spline",
    "fit_open_spline",
    "check_spline_coeffs_dims",
    "compute_spline_interval_lengths",
    "uniformly_sample_spline",
    "get_heading",
    "get_curvature",
]


def fit_closed_spline(
    path: FloatArray,
    curv_weight: float = 1.0,
) -> tuple[FloatArray, FloatArray]:
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
    assert (
        len(path.shape) == 2 and path.shape[1] == 2
    ), f"path must have shape (N,2) but has shape {path.shape}"

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
                    (3 * np.arange(N), 1 + 3 * np.arange(N), 2 + 3 * np.arange(N))
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
    P = B.T @ B + curv_weight * C.T @ C
    q = -B.T @ path
    b = np.zeros(3 * N)

    # solve the QP for X and Y separately
    p_X = solve_qp(P=P, q=q[:, 0], A=A, b=b, solver="osqp")
    p_Y = solve_qp(P=P, q=q[:, 1], A=A, b=b, solver="osqp")
    if p_X is None or p_Y is None:
        raise ValueError("solving qp failed")

    # reshape to (N,4) arrays
    p_X = np.reshape(p_X, (N, 4))
    p_Y = np.reshape(p_Y, (N, 4))

    return p_X, p_Y


def fit_open_spline(
    path: FloatArray,
    curv_weight: float = 1.0,
    initial_heading: float = np.pi / 2,
    final_heading: float = np.pi / 2,
):
    assert (
        len(path.shape) == 2 and path.shape[1] == 2
    ), f"path must have shape (N,2) but has shape {path.shape}"

    # precompute all the QP data
    N = path.shape[0] - 1  # path has N+1 points for N segments
    delta_s = np.linalg.norm(path[1:] - path[:-1], axis=1)
    rho = delta_s[:-1] / delta_s[1:]
    A = vstack(
        (
            hstack(
                (
                    spkron(
                        speye(N - 1, format="csc"),
                        np.array(
                            [
                                [1.0, 1.0, 1.0, 1.0],
                                [0.0, 1.0, 2.0, 3.0],
                                [0.0, 0.0, 2.0, 6.0],
                            ]
                        ),
                        format="csc",
                    ),
                    csc_array((3 * (N - 1), 4), dtype=np.float64),
                ),
                format="csc",
            )
            + csc_array(
                (
                    np.concatenate((-np.ones(N - 1), -rho, -2 * rho**2)),
                    (
                        np.concatenate(
                            (
                                3 * np.arange(N - 1),
                                1 + 3 * np.arange(N - 1),
                                2 + 3 * np.arange(N - 1),
                            )
                        ),
                        np.concatenate(
                            (
                                4 + 4 * np.arange(N - 1),
                                5 + 4 * np.arange(N - 1),
                                6 + 4 * np.arange(N - 1),
                            )
                        ),
                    ),
                ),
                shape=(3 * (N - 1), 4 * N),
            ),
            spkron(
                np.append(1.0, np.zeros(N - 1)),
                np.array([[0.0, 1.0, 0.0, 0.0]]),
                format="csc",
            ),
            spkron(
                np.append(np.zeros(N - 1), 1.0),
                np.array([[0.0, 1.0, 2.0, 3.0]]),
                format="csc",
            ),
        ),
        format="csc",
    )
    B = vstack(
        (
            spkron(
                speye(N, format="csc"), np.array([[1.0, 0.0, 0.0, 0.0]]), format="csc"
            ),
            spkron(np.append(np.zeros(N - 1), 1.0), np.ones(4), format="csc"),
        ),
        format="csc",
    )
    C = vstack(
        (
            csc_array(
                (2 / np.square(delta_s), (np.arange(N), 2 + 4 * np.arange(N))),
                shape=(N, 4 * N),
            ),
            spkron(
                np.append(np.zeros(N - 1), 1.0),
                np.array(
                    [[0.0, 0.0, 2.0 / (delta_s[-1] ** 2), 6.0 / (delta_s[-1] ** 2)]]
                ),
                format="csc",
            ),
        ),
        format="csc",
    )

    P = B.T @ B + curv_weight * C.T @ C
    q = -B.T @ path

    # solve the QP for X and Y separately
    p_X = solve_qp(
        P=P,
        q=q[:, 0],
        A=A,
        b=np.concatenate(
            (
                np.zeros(3 * (N - 1)),
                np.array(
                    [
                        delta_s[0] * np.cos(initial_heading),
                        delta_s[-1] * np.cos(final_heading),
                    ]
                ),
            )
        ),
        solver="osqp",
    )
    p_Y = solve_qp(
        P=P,
        q=q[:, 1],
        A=A,
        b=np.concatenate(
            (
                np.zeros(3 * (N - 1)),
                np.array(
                    [
                        delta_s[0] * np.sin(initial_heading),
                        delta_s[-1] * np.sin(final_heading),
                    ]
                ),
            )
        ),
        solver="osqp",
    )
    if p_X is None or p_Y is None:
        raise ValueError("solving qp failed")

    # reshape to (N,4) arrays
    p_X = np.reshape(p_X, (N, 4))
    p_Y = np.reshape(p_Y, (N, 4))

    return p_X, p_Y


def check_spline_coeffs_dims(coeffs_X: FloatArray, coeffs_Y: FloatArray):
    assert (
        len(coeffs_X.shape) == 2 and coeffs_X.shape[1] == 4
    ), f"coeffs_X must have shape (N,4) but has shape {coeffs_X.shape}"
    assert (
        len(coeffs_Y.shape) == 2 and coeffs_Y.shape[1] == 4
    ), f"coeffs_Y must have shape (N,4) but has shape {coeffs_Y.shape}"
    assert (
        coeffs_X.shape[0] == coeffs_Y.shape[0]
    ), f"coeffs_X and coeffs_Y must have the same length but have lengths {coeffs_X.shape[0]} and {coeffs_Y.shape[0]}"


def compute_spline_interval_lengths(
    coeffs_X: FloatArray, coeffs_Y: FloatArray, no_interp_points=100
) -> FloatArray:
    """
    computes the lengths of each spline portion of the path.
    > Note: Here the closeness of the part does not matter, it is contained in the coefficients

    :param coeff_X: Nx4 array of coefficients of the splines in the x direction (as returned by fit_spline)
    :param coeff_Y: Nx4 array of coefficients of the splines in the y direction (as returned by fit_spline)
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
    coeffs_X: FloatArray,
    coeffs_Y: FloatArray,
    delta_s: FloatArray,
    n_samples: int,
) -> tuple[FloatArray, FloatArray, IntArray, FloatArray, FloatArray]:
    """
    uniformly n_samples equidistant points along the path defined by the splines.
    The first point will always be the initial point of the first spline portion, and
    the last point will NOT be the initial point of the first spline portion.

    :param coeffs_X: Nx4 array of coefficients of the splines in the x direction (as returned by fit_spline)
    :param coeffs_Y: Nx4 array of coefficients of the splines in the y direction (as returned by fit_spline)
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
    coeffs_X: FloatArray,
    coeffs_Y: FloatArray,
    idx_interp: npt.NDArray[np.int64],
    t_interp: FloatArray,
) -> FloatArray:
    """
    analytically computes the heading and the curvature at each point along the path
    specified by idx_interp and t_interp.

    :param coeffs_X: Nx4 array of coefficients of the splines in the x direction (as returned by fit_spline)
    :param coeffs_Y: Nx4 array of coefficients of the splines in the y direction (as returned by fit_spline)
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
    coeffs_X: FloatArray,
    coeffs_Y: FloatArray,
    idx_interp: npt.NDArray[np.int64],
    t_interp: FloatArray,
) -> FloatArray:
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
