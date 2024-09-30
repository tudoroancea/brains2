// SplineFitter.cpp

#include "brains2/estimation/spline_fitting/spline_fitter.h"
#include <cmath>
#include <numeric>
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
        return tl::make_unexpected(SplineFittingError::NotEnoughPoints);
    }

    return SplineFitter(path, curv_weight, verbose);
}

tl::expected<void, SplineFittingError> SplineFitter::fit_open_spline() {
    // Ensure the path has shape (N+1, 2)
    if (path.cols() != 2) {
        return tl::make_unexpected(SplineFittingError::PathShape);
    }
    int N = path.rows() - 1;  // path has N+1 points for N segments

    // Compute delta_s
    Eigen::MatrixXd delta_diff = path.block(1, 0, N, 2) - path.block(0, 0, N, 2);
    Eigen::VectorXd delta_s = delta_diff.rowwise().norm();

    // Compute rho
    Eigen::VectorXd rho = delta_s.head(N - 1).array() / delta_s.tail(N - 1).array();

    // Build matrices A, B, C

    // Build A
    std::vector<Eigen::Triplet<double>> tripletListA;
    tripletListA.reserve(9 * (N - 1) + 3 * (N - 1) + 4);

    // First part of A: spkron(speye(N - 1), array)
    for (int i = 0; i < N - 1; ++i) {
        int row_offset = 3 * i;
        int col_offset = 4 * i;

        // First row : continuity between spline segments
        tripletListA.emplace_back(row_offset, col_offset + 0, 1.0);
        tripletListA.emplace_back(row_offset, col_offset + 1, 1.0);
        tripletListA.emplace_back(row_offset, col_offset + 2, 1.0);
        tripletListA.emplace_back(row_offset, col_offset + 3, 1.0);

        // Second row : continuity of first derivative
        tripletListA.emplace_back(row_offset + 1, col_offset + 1, 1.0);
        tripletListA.emplace_back(row_offset + 1, col_offset + 2, 2.0);
        tripletListA.emplace_back(row_offset + 1, col_offset + 3, 3.0);

        // Third row : continuity of second derivative
        tripletListA.emplace_back(row_offset + 2, col_offset + 2, 2.0);
        tripletListA.emplace_back(row_offset + 2, col_offset + 3, 6.0);
    }

    // Second part of A: additional entries based on rho
    for (int i = 0; i < N - 1; ++i) {
        int row = 3 * i;
        int col = 4 * (i + 1);

        // Entries for -1, -rho, -2*rho^2
        tripletListA.emplace_back(row, col + 0, -1.0);
        tripletListA.emplace_back(row + 1, col + 1, -rho[i]);
        tripletListA.emplace_back(row + 2, col + 2, -2.0 * rho[i] * rho[i]);
    }

    // Third part of A: initial heading constraint
    tripletListA.emplace_back(3 * (N - 1), 1, 1.0);

    // Fourth part of A: final heading constraint
    int last_row = 3 * (N - 1) + 1;
    int col_offset = 4 * (N - 1);
    tripletListA.emplace_back(last_row, col_offset + 1, 1.0);
    tripletListA.emplace_back(last_row, col_offset + 2, 2.0);
    tripletListA.emplace_back(last_row, col_offset + 3, 3.0);

    int A_rows = 3 * (N - 1) + 2;
    int A_cols = 4 * N;
    Eigen::SparseMatrix<double> A(A_rows, A_cols);
    A.setFromTriplets(tripletListA.begin(), tripletListA.end());

    // Build B
    std::vector<Eigen::Triplet<double>> tripletListB;
    tripletListB.reserve(N + 4);

    // First part of B: spkron(speye(N), [1, 0, 0, 0])
    for (int i = 0; i < N; ++i) {
        int row = i;
        int col = 4 * i;
        tripletListB.emplace_back(row, col, 1.0);
    }

    // Second part of B: spkron([0, ..., 0, 1], [1, 1, 1, 1])
    int last_row_B = N;
    col_offset = 4 * (N - 1);
    for (int j = 0; j < 4; ++j) {
        tripletListB.emplace_back(last_row_B, col_offset + j, 1.0);
    }

    int B_rows = N + 1;
    int B_cols = 4 * N;
    Eigen::SparseMatrix<double> B(B_rows, B_cols);
    B.setFromTriplets(tripletListB.begin(), tripletListB.end());

    // Build C
    std::vector<Eigen::Triplet<double>> tripletListC;
    tripletListC.reserve(N + 2);

    // First part of C
    for (int i = 0; i < N; ++i) {
        int row = i;
        int col = 4 * i + 2;
        double value = 2.0 / (delta_s[i] * delta_s[i]);
        tripletListC.emplace_back(row, col, value);
    }

    // Second part of C: last row
    double delta_s_last = delta_s[N - 1];
    int last_row_C = N;
    col_offset = 4 * (N - 1);
    tripletListC.emplace_back(last_row_C, col_offset + 2, 2.0 / (delta_s_last * delta_s_last));
    tripletListC.emplace_back(last_row_C, col_offset + 3, 6.0 / (delta_s_last * delta_s_last));

    int C_rows = N + 1;
    int C_cols = 4 * N;
    Eigen::SparseMatrix<double> C(C_rows, C_cols);
    C.setFromTriplets(tripletListC.begin(), tripletListC.end());

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
        return tl::make_unexpected(SplineFittingError::SetHessian);
    }
    if (!solver.data()->setGradient(q.col(0))) {
        return tl::make_unexpected(SplineFittingError::SetGradient);
    }
    if (!solver.data()->setLinearConstraintsMatrix(A)) {
        return tl::make_unexpected(SplineFittingError::SetConstraints);
    }
    solver.data()->setBounds(b_x, b_x);
    if (!solver.initSolver()) {
        return tl::make_unexpected(SplineFittingError::InitSolver);
    }

    // Solve for X
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        return tl::make_unexpected(SplineFittingError::SolveQP);
    }

    // Eigen::VectorXd p_X_vec = solver.getSolution();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> p_X =
        solver.getSolution();

    if (!solver.updateGradient(q.col(1))) {
        return tl::make_unexpected(SplineFittingError::SetGradient);
    }
    if (!solver.updateLinearConstraintsMatrix(A)) {
        return tl::make_unexpected(SplineFittingError::SetConstraints);
    }
    solver.updateBounds(b_y, b_y);

    // Solve for Y
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        return tl::make_unexpected(SplineFittingError::SolveQP);
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
        return tl::make_unexpected(SplineFittingError::CoefficientsDimensions);
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
        return tl::make_unexpected(SplineFittingError::CoefficientsDimensions);
    }
    if (delta_s.size() != N) {
        return tl::make_unexpected(SplineFittingError::DeltaSLength);
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

const MatrixPair& SplineFitter::get_spline_coefs() const {
    return spline_coefficients;
}

const Eigen::VectorXd& SplineFitter::get_delta_s() const {
    return delta_s;
}