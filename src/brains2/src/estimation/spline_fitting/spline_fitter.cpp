// SplineFitter.cpp

#include "brains2/estimation/spline_fitting/spline_fitter.h"
#include <cmath>
#include <numeric>
#include <set>
#include <vector>
#include "brains2/external/icecream.hpp"

SplineFitter::SplineFitter(const Eigen::MatrixXd& path, double curv_weight, bool verbose)
    : path(path), curv_weight(curv_weight), verbose(verbose) {
    initial_heading = atan2(path(1, 1) - path(0, 1), path(1, 0) - path(0, 0));
    final_heading = atan2(path(path.rows() - 1, 1) - path(path.rows() - 2, 1),
                          path(path.rows() - 1, 0) - path(path.rows() - 2, 0));
}

tl::expected<SplineFitter, SplineFittingError> SplineFitter::create(const Eigen::MatrixXd& path,
                                                                    double curv_weight,
                                                                    bool verbose) {
    // Ensure the path has at least 3 points
    if (path.rows() < 3) {
        return tl::make_unexpected(SplineFittingError::NOT_ENOUGH_POINTS);
    }

    return SplineFitter(path, curv_weight, verbose);
}

tl::expected<void, SplineFittingError> SplineFitter::fit_open_spline() {
    // Ensure the path has shape (N+1, 2)
    if (path.cols() != 2) {
        return tl::make_unexpected(SplineFittingError::PATH_SHAPE);
    }
    int N = path.rows() - 1;  // path has N+1 points for N segments

    // Compute delta_s
    Eigen::MatrixXd delta_diff = path.block(1, 0, N, 2) - path.block(0, 0, N, 2);
    Eigen::VectorXd delta_s = delta_diff.rowwise().norm();

    // Compute rho
    Eigen::VectorXd rho = delta_s.head(N - 1).array() / delta_s.tail(N - 1).array();

    // Build matrices A, B, C

    // Build A
    std::vector<Eigen::Triplet<double>> triplet_list_A;
    triplet_list_A.reserve(9 * (N - 1) + 3 * (N - 1) + 4);

    // First part of A: spkron(speye(N - 1), array)
    for (int i = 0; i < N - 1; ++i) {
        int row_offset = 3 * i;
        int col_offset = 4 * i;

        // First row : continuity between spline segments
        triplet_list_A.emplace_back(row_offset, col_offset + 0, 1.0);
        triplet_list_A.emplace_back(row_offset, col_offset + 1, 1.0);
        triplet_list_A.emplace_back(row_offset, col_offset + 2, 1.0);
        triplet_list_A.emplace_back(row_offset, col_offset + 3, 1.0);

        // Second row : continuity of first derivative
        triplet_list_A.emplace_back(row_offset + 1, col_offset + 1, 1.0);
        triplet_list_A.emplace_back(row_offset + 1, col_offset + 2, 2.0);
        triplet_list_A.emplace_back(row_offset + 1, col_offset + 3, 3.0);

        // Third row : continuity of second derivative
        triplet_list_A.emplace_back(row_offset + 2, col_offset + 2, 2.0);
        triplet_list_A.emplace_back(row_offset + 2, col_offset + 3, 6.0);
    }

    // Second part of A: additional entries based on rho
    for (int i = 0; i < N - 1; ++i) {
        int row = 3 * i;
        int col = 4 * (i + 1);

        // Entries for -1, -rho, -2*rho^2
        triplet_list_A.emplace_back(row, col + 0, -1.0);
        triplet_list_A.emplace_back(row + 1, col + 1, -rho[i]);
        triplet_list_A.emplace_back(row + 2, col + 2, -2.0 * rho[i] * rho[i]);
    }

    // Third part of A: initial heading constraint
    triplet_list_A.emplace_back(3 * (N - 1), 1, 1.0);

    // Fourth part of A: final heading constraint
    int last_row = 3 * (N - 1) + 1;
    int col_offset = 4 * (N - 1);
    triplet_list_A.emplace_back(last_row, col_offset + 1, 1.0);
    triplet_list_A.emplace_back(last_row, col_offset + 2, 2.0);
    triplet_list_A.emplace_back(last_row, col_offset + 3, 3.0);

    int A_rows = 3 * (N - 1) + 2;
    int A_cols = 4 * N;
    Eigen::SparseMatrix<double> A(A_rows, A_cols);
    A.setFromTriplets(triplet_list_A.begin(), triplet_list_A.end());

    // Build B
    std::vector<Eigen::Triplet<double>> triplet_list_B;
    triplet_list_B.reserve(N + 4);

    // First part of B: spkron(speye(N), [1, 0, 0, 0])
    for (int i = 0; i < N; ++i) {
        int row = i;
        int col = 4 * i;
        triplet_list_B.emplace_back(row, col, 1.0);
    }

    // Second part of B: spkron([0, ..., 0, 1], [1, 1, 1, 1])
    int last_row_B = N;
    col_offset = 4 * (N - 1);
    for (int j = 0; j < 4; ++j) {
        triplet_list_B.emplace_back(last_row_B, col_offset + j, 1.0);
    }

    int B_rows = N + 1;
    int B_cols = 4 * N;
    Eigen::SparseMatrix<double> B(B_rows, B_cols);
    B.setFromTriplets(triplet_list_B.begin(), triplet_list_B.end());

    // Build C
    std::vector<Eigen::Triplet<double>> triplet_list_C;
    triplet_list_C.reserve(N + 2);

    // First part of C
    for (int i = 0; i < N; ++i) {
        int row = i;
        int col = 4 * i + 2;
        double value = 2.0 / (delta_s[i] * delta_s[i]);
        triplet_list_C.emplace_back(row, col, value);
    }

    // Second part of C: last row
    double delta_s_last = delta_s[N - 1];
    int last_row_C = N;
    col_offset = 4 * (N - 1);
    triplet_list_C.emplace_back(last_row_C, col_offset + 2, 2.0 / (delta_s_last * delta_s_last));
    triplet_list_C.emplace_back(last_row_C, col_offset + 3, 6.0 / (delta_s_last * delta_s_last));

    int C_rows = N + 1;
    int C_cols = 4 * N;
    Eigen::SparseMatrix<double> C(C_rows, C_cols);
    C.setFromTriplets(triplet_list_C.begin(), triplet_list_C.end());

    // Compute P = B^T * B + curv_weight * C^T * C
    Eigen::SparseMatrix<double> P = B.transpose() * B + curv_weight * C.transpose() * C;

    // Compute q = -B^T * path_
    Eigen::MatrixXd q = -B.transpose() * path;

    // Build constraint vector b
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(A_rows);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(A_rows);

    b_x[A_rows - 2] = delta_s[0] * std::cos(initial_heading);
    b_x[A_rows - 1] = delta_s[N - 1] * std::cos(final_heading);

    b_y[A_rows - 2] = delta_s[0] * std::sin(initial_heading);
    b_y[A_rows - 1] = delta_s[N - 1] * std::sin(final_heading);

    // Setup OSQP solver
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(verbose);
    solver.data()->setNumberOfVariables(P.cols());
    solver.data()->setNumberOfConstraints(A_rows);

    if (!solver.data()->setHessianMatrix(P)) {
        return tl::make_unexpected(SplineFittingError::SET_HESSIAN);
    }
    if (!solver.data()->setGradient(q.col(0))) {
        return tl::make_unexpected(SplineFittingError::SET_GRADIENT);
    }
    if (!solver.data()->setLinearConstraintsMatrix(A)) {
        return tl::make_unexpected(SplineFittingError::SET_CONSTRAINTS);
    }
    solver.data()->setBounds(b_x, b_x);
    if (!solver.initSolver()) {
        return tl::make_unexpected(SplineFittingError::INIT_SOLVER);
    }

    // Solve for X
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        return tl::make_unexpected(SplineFittingError::SOLVE_QP);
    }

    // Eigen::VectorXd p_X_vec = solver.getSolution();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> p_X =
        solver.getSolution();

    if (!solver.updateGradient(q.col(1))) {
        return tl::make_unexpected(SplineFittingError::SET_GRADIENT);
    }
    if (!solver.updateLinearConstraintsMatrix(A)) {
        return tl::make_unexpected(SplineFittingError::SET_CONSTRAINTS);
    }
    solver.updateBounds(b_y, b_y);

    // Solve for Y
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        return tl::make_unexpected(SplineFittingError::SOLVE_QP);
    }

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> p_Y =
        solver.getSolution();

    // return std::make_pair(p_X_mat, p_Y_mat);
    p_X.resize(N, 4);
    p_Y.resize(N, 4);

    spline_coefficients = std::make_pair(p_X, p_Y);

    return {};
}

tl::expected<void, SplineFittingError> SplineFitter::compute_spline_interval_lengths(
    int no_interp_points) {
    // Get coefficients
    const Eigen::MatrixXd& coeffs_X = spline_coefficients.first;
    const Eigen::MatrixXd& coeffs_Y = spline_coefficients.second;

    // Check that coeffs_X and coeffs_Y have the same dimensions and are Nx4 matrices
    if (coeffs_X.rows() != coeffs_Y.rows() || coeffs_X.cols() != 4 || coeffs_Y.cols() != 4) {
        return tl::make_unexpected(SplineFittingError::COEFFICIENTS_DIMENSIONS);
    }

    int N = coeffs_X.rows();  // Number of spline segments

    // Generate t_steps from 0.0 to 1.0 with no_interp_points points
    Eigen::VectorXd t_steps = Eigen::VectorXd::LinSpaced(no_interp_points, 0.0, 1.0);
    Eigen::VectorXd t_steps_square = t_steps.array().square();
    Eigen::VectorXd t_steps_cube = t_steps.array().cube();

    // Initialize delta_s vector to hold lengths of each spline interval
    delta_s = Eigen::VectorXd(N);

    // Loop over each spline segment
    for (int i = 0; i < N; ++i) {
        // Get the coefficients for the current spline segment
        Eigen::VectorXd coeff_X = coeffs_X.row(i);
        Eigen::VectorXd coeff_Y = coeffs_Y.row(i);

        // Evaluate the spline at the t_steps
        // x(t) = a0 + a1*t + a2*t^2 + a3*t^3
        Eigen::VectorXd x = Eigen::VectorXd::Zero(no_interp_points);
        Eigen::VectorXd y = Eigen::VectorXd::Zero(no_interp_points);

        // Add contributions from each term
        x.array() += coeff_X(0);                           // a0
        x.array() += coeff_X(1) * t_steps.array();         // a1 * t
        x.array() += coeff_X(2) * t_steps_square.array();  // a2 * t^2
        x.array() += coeff_X(3) * t_steps_cube.array();    // a3 * t^3

        y.array() += coeff_Y(0);                           // b0
        y.array() += coeff_Y(1) * t_steps.array();         // b1 * t
        y.array() += coeff_Y(2) * t_steps_square.array();  // b2 * t^2
        y.array() += coeff_Y(3) * t_steps_cube.array();    // b3 * t^3

        // Compute differences between consecutive points
        Eigen::VectorXd dx =
            x.segment(1, no_interp_points - 1) - x.segment(0, no_interp_points - 1);
        Eigen::VectorXd dy =
            y.segment(1, no_interp_points - 1) - y.segment(0, no_interp_points - 1);

        // Compute distances between consecutive points
        Eigen::VectorXd ds = (dx.array().square() + dy.array().square()).sqrt();

        // Sum distances to get the approximate length of the spline segment
        delta_s(i) = ds.sum();
    }

    return {};
}

tl::expected<
    std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXi, Eigen::VectorXd, Eigen::VectorXd>,
    SplineFittingError>
SplineFitter::uniformly_sample_spline(int n_samples) {
    // n_samples =
    //     n_samples - 1;  // Subtract 1 to exclude the last point which will be added at the end
    // Get coefficients
    const Eigen::MatrixXd& coeffs_X = spline_coefficients.first;
    const Eigen::MatrixXd& coeffs_Y = spline_coefficients.second;

    // Check that coeffs_X and coeffs_Y have shape (N, 4)
    int N = coeffs_X.rows();
    if (coeffs_X.rows() != coeffs_Y.rows() || coeffs_X.cols() != 4 || coeffs_Y.cols() != 4) {
        return tl::make_unexpected(SplineFittingError::COEFFICIENTS_DIMENSIONS);
    }
    if (delta_s.size() != N) {
        return tl::make_unexpected(SplineFittingError::DELTA_S_LENGTH);
    }

    // Compute cumulative sum of delta_s_
    Eigen::VectorXd s(N);
    std::partial_sum(delta_s.data(), delta_s.data() + N, s.data());

    // Generate s_interp: n_samples equally spaced values from 0 to s(N - 1), excluding endpoint
    double s_max = s(N - 1);
    Eigen::VectorXd s_interp = Eigen::VectorXd::LinSpaced(n_samples, 0.0, s_max);

    // Find idx_interp: index of the spline segment that contains each s_interp[i]
    Eigen::VectorXi idx_interp(n_samples);
    for (int i = 0; i < n_samples; ++i) {
        auto it = std::upper_bound(s.data(), s.data() + N, s_interp(i));
        int idx = static_cast<int>(it - s.data());
        if (idx >= N) {
            idx = N - 1;
        }
        idx_interp(i) = idx;
    }

    // Compute t_interp: parameter t within each spline segment
    Eigen::VectorXd t_interp(n_samples);
    for (int i = 0; i < n_samples; ++i) {
        int idx = idx_interp(i);
        if (idx > 0) {
            t_interp(i) = (s_interp(i) - s(idx - 1)) / delta_s(idx);
        } else {
            t_interp(i) = s_interp(i) / delta_s(0);
        }
        if (t_interp(i) > 1.0) {
            t_interp(i) = 1.0;
        }
    }

    // Evaluate X_interp and Y_interp using the spline coefficients
    Eigen::VectorXd X_interp(n_samples);
    Eigen::VectorXd Y_interp(n_samples);
    for (int i = 0; i < n_samples; ++i) {
        int idx = idx_interp(i);

        double t = t_interp(i);
        double t2 = t * t;
        double t3 = t2 * t;

        X_interp(i) =
            coeffs_X(idx, 0) + coeffs_X(idx, 1) * t + coeffs_X(idx, 2) * t2 + coeffs_X(idx, 3) * t3;
        Y_interp(i) =
            coeffs_Y(idx, 0) + coeffs_Y(idx, 1) * t + coeffs_Y(idx, 2) * t2 + coeffs_Y(idx, 3) * t3;
    }

    return std::make_tuple(X_interp, Y_interp, idx_interp, t_interp, s_interp);
}

// Function to compute heading (phi)
tl::expected<Eigen::VectorXd, SplineFittingError> SplineFitter::get_heading(
    const Eigen::VectorXi& idx_interp, const Eigen::VectorXd& t_interp) {
    auto coeffs_X = spline_coefficients.first;
    auto coeffs_Y = spline_coefficients.second;
    // Ensure idx_interp and t_interp have the same size
    if (idx_interp.size() != t_interp.size()) {
        return tl::unexpected(SplineFittingError::SIZE_MISMATCH);
    }

    // Check index bounds
    if ((idx_interp.array() < 0).any() || (idx_interp.array() >= coeffs_X.rows()).any()) {
        return tl::unexpected(SplineFittingError::INDEX_OUT_OF_BOUNDS);
    }

    int n_samples = idx_interp.size();
    Eigen::VectorXd c1x(n_samples), c2x(n_samples), c3x(n_samples);
    Eigen::VectorXd c1y(n_samples), c2y(n_samples), c3y(n_samples);

    // Extract coefficients based on idx_interp
    for (int i = 0; i < n_samples; ++i) {
        int idx = idx_interp(i);
        c1x(i) = coeffs_X(idx, 1);
        c2x(i) = coeffs_X(idx, 2);
        c3x(i) = coeffs_X(idx, 3);
        c1y(i) = coeffs_Y(idx, 1);
        c2y(i) = coeffs_Y(idx, 2);
        c3y(i) = coeffs_Y(idx, 3);
    }

    Eigen::VectorXd t_sq = t_interp.array().square();

    // Compute first derivatives x_d and y_d
    Eigen::VectorXd x_d =
        c1x.array() + 2.0 * c2x.array() * t_interp.array() + 3.0 * c3x.array() * t_sq.array();
    Eigen::VectorXd y_d =
        c1y.array() + 2.0 * c2y.array() * t_interp.array() + 3.0 * c3y.array() * t_sq.array();

    // Compute heading phi
    Eigen::VectorXd phi(n_samples);
    for (int i = 0; i < n_samples; ++i) {
        phi(i) = std::atan2(y_d(i), x_d(i));
    }

    return phi;
}

// Function to compute curvature (kappa)
tl::expected<Eigen::VectorXd, SplineFittingError> SplineFitter::get_curvature(
    const Eigen::VectorXi& idx_interp, const Eigen::VectorXd& t_interp) {
    auto coeffs_X = spline_coefficients.first;
    auto coeffs_Y = spline_coefficients.second;

    // Ensure idx_interp and t_interp have the same size
    if (idx_interp.size() != t_interp.size()) {
        return tl::unexpected(SplineFittingError::SIZE_MISMATCH);
    }

    // Check index bounds
    if ((idx_interp.array() < 0).any() ||
        (idx_interp.array() >= spline_coefficients.first.rows()).any()) {
        return tl::unexpected(SplineFittingError::INDEX_OUT_OF_BOUNDS);
    }

    int n_samples = idx_interp.size();
    Eigen::VectorXd c1x(n_samples), c2x(n_samples), c3x(n_samples);
    Eigen::VectorXd c1y(n_samples), c2y(n_samples), c3y(n_samples);

    // Extract coefficients based on idx_interp
    for (int i = 0; i < n_samples; ++i) {
        int idx = idx_interp(i);
        c1x(i) = coeffs_X(idx, 1);
        c2x(i) = coeffs_X(idx, 2);
        c3x(i) = coeffs_X(idx, 3);
        c1y(i) = coeffs_Y(idx, 1);
        c2y(i) = coeffs_Y(idx, 2);
        c3y(i) = coeffs_Y(idx, 3);
    }

    Eigen::VectorXd t_sq = t_interp.array().square();

    // Compute first derivatives x_d and y_d
    Eigen::VectorXd x_d =
        c1x.array() + 2.0 * c2x.array() * t_interp.array() + 3.0 * c3x.array() * t_sq.array();
    Eigen::VectorXd y_d =
        c1y.array() + 2.0 * c2y.array() * t_interp.array() + 3.0 * c3y.array() * t_sq.array();

    // Compute second derivatives x_dd and y_dd
    Eigen::VectorXd x_dd = 2.0 * c2x.array() + 6.0 * c3x.array() * t_interp.array();
    Eigen::VectorXd y_dd = 2.0 * c2y.array() + 6.0 * c3y.array() * t_interp.array();

    // Compute numerator and denominator for curvature formula
    Eigen::VectorXd numerator = x_d.array() * y_dd.array() - y_d.array() * x_dd.array();
    Eigen::VectorXd denominator = (x_d.array().square() + y_d.array().square()).array().pow(1.5);

    // Handle potential zero denominators
    const double epsilon = 1e-8;
    if ((denominator.array().abs() < epsilon).any()) {
        return tl::unexpected(SplineFittingError::ZERO_DENOMINATOR);
    }

    // Compute curvature kappa
    Eigen::VectorXd kappa = numerator.array() / denominator.array();

    return kappa;
}

// tl::expected<VectorPair, SplineFittingError> SplineFitter::compute_center_line(
//     Eigen::VectorXd X_blue,
//     Eigen::VectorXd Y_blue,
//     Eigen::VectorXd X_yellow,
//     Eigen::VectorXd Y_yellow) {
//     long sizes[4] = {X_blue.size(), Y_blue.size(), X_yellow.size(), Y_yellow.size()};
//     // check if all sizes are equal
//     if (not std::all_of(std::begin(sizes), std::end(sizes), [&](long size) {
//             return size == sizes[0];
//         })) {
//         return tl::make_unexpected(SplineFittingError::SIZE_MISMATCH);
//     }

//     Eigen::VectorXd X_center = (X_blue + X_yellow) / 2;
//     Eigen::VectorXd Y_center = (Y_blue + Y_yellow) / 2;

//     return std::make_pair(X_center, Y_center);
// }

tl::expected<VectorPair, SplineFittingError> SplineFitter::compute_center_line(
    const Eigen::VectorXd& X_blue,
    const Eigen::VectorXd& Y_blue,
    const Eigen::VectorXd& X_yellow,
    const Eigen::VectorXd& Y_yellow,
    double curv_weight,
    bool verbose) {
    // Check that sizes of X and Y are equal for each boundary
    if (X_blue.size() != Y_blue.size() || X_yellow.size() != Y_yellow.size()) {
        return tl::make_unexpected(SplineFittingError::SIZE_MISMATCH);
    }

    using IndexPair = std::pair<int, int>;
    std::set<IndexPair> matched_pairs;

    // Find closest points from blue to yellow boundary
    for (int i = 0; i < X_blue.size(); ++i) {
        Eigen::Vector2d blue_point(X_blue[i], Y_blue[i]);

        double min_dist_sq = std::numeric_limits<double>::max();
        int min_index = -1;
        for (int j = 0; j < X_yellow.size(); ++j) {
            Eigen::Vector2d yellow_point(X_yellow[j], Y_yellow[j]);
            double dist_sq = (blue_point - yellow_point).squaredNorm();
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                min_index = j;
            }
        }

        matched_pairs.insert({i, min_index});
    }

    // Find closest points from yellow to blue boundary
    for (int j = 0; j < X_yellow.size(); ++j) {
        Eigen::Vector2d yellow_point(X_yellow[j], Y_yellow[j]);

        double min_dist_sq = std::numeric_limits<double>::max();
        int min_index = -1;
        for (int i = 0; i < X_blue.size(); ++i) {
            Eigen::Vector2d blue_point(X_blue[i], Y_blue[i]);
            double dist_sq = (yellow_point - blue_point).squaredNorm();
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                min_index = i;
            }
        }

        matched_pairs.insert({min_index, j});
    }

    // Precompute cumulative distances along blue and yellow boundaries
    std::vector<double> cumulative_distances_blue(X_blue.size(), 0.0);
    for (int i = 1; i < X_blue.size(); ++i) {
        Eigen::Vector2d prev_point(X_blue[i - 1], Y_blue[i - 1]);
        Eigen::Vector2d curr_point(X_blue[i], Y_blue[i]);
        cumulative_distances_blue[i] =
            cumulative_distances_blue[i - 1] + (curr_point - prev_point).norm();
    }

    std::vector<double> cumulative_distances_yellow(X_yellow.size(), 0.0);
    for (int j = 1; j < X_yellow.size(); ++j) {
        Eigen::Vector2d prev_point(X_yellow[j - 1], Y_yellow[j - 1]);
        Eigen::Vector2d curr_point(X_yellow[j], Y_yellow[j]);
        cumulative_distances_yellow[j] =
            cumulative_distances_yellow[j - 1] + (curr_point - prev_point).norm();
    }

    // Compute middle points and parameters for ordering
    std::vector<std::pair<Eigen::Vector2d, double>> center_points_with_param;
    for (const auto& [i, j] : matched_pairs) {
        Eigen::Vector2d blue_point(X_blue[i], Y_blue[i]);
        Eigen::Vector2d yellow_point(X_yellow[j], Y_yellow[j]);
        Eigen::Vector2d center_point = (blue_point + yellow_point) / 2.0;

        // Average cumulative distance for ordering
        double param = (cumulative_distances_blue[i] + cumulative_distances_yellow[j]) / 2.0;
        center_points_with_param.emplace_back(center_point, param);
    }

    // Sort center points based on the computed parameter
    std::sort(center_points_with_param.begin(),
              center_points_with_param.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });

    // Extract X_center and Y_center

    Eigen::MatrixXd path = Eigen::MatrixXd(center_points_with_param.size(), 2);
    for (size_t idx = 0; idx < center_points_with_param.size(); ++idx) {
        path(idx, 0) = center_points_with_param[idx].first.x();
        path(idx, 1) = center_points_with_param[idx].first.y();
    }

    // Eigen::VectorXd X_center(center_points_with_param.size());
    // Eigen::VectorXd Y_center(center_points_with_param.size());
    // for (size_t idx = 0; idx < center_points_with_param.size(); ++idx) {
    //     X_center[idx] = center_points_with_param[idx].first.x();
    //     Y_center[idx] = center_points_with_param[idx].first.y();
    // }

    // Fit a spline to the center points
    auto expected_center_line = SplineFitter::create(path, curv_weight, verbose);
    if (!expected_center_line) {
        return tl::make_unexpected(expected_center_line.error());
    }

    SplineFitter center_line_fitter = expected_center_line.value();
    auto fit_result = center_line_fitter.fit_open_spline();

    if (!fit_result) {
        return tl::make_unexpected(fit_result.error());
    }

    auto length_result = center_line_fitter.compute_spline_interval_lengths();

    if (!length_result) {
        return tl::make_unexpected(length_result.error());
    }

    auto sample_result = center_line_fitter.uniformly_sample_spline(300);

    if (!sample_result) {
        return tl::make_unexpected(sample_result.error());
    }

    auto [X_center, Y_center, idx_interp, t_interp, s_interp] = sample_result.value();

    return std::make_pair(X_center, Y_center);
}

const MatrixPair& SplineFitter::get_spline_coefs() const {
    return spline_coefficients;
}

const Eigen::VectorXd& SplineFitter::get_delta_s() const {
    return delta_s;
}