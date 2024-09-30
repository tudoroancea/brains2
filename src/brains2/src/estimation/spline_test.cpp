#include <osqp/osqp.h>
#include <OsqpEigen/OsqpEigen.h>
#include <algorithm>  // for std::upper_bound
#include <cassert>
#include <chrono>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <fstream>
#include <iostream>
#include <numeric>  // for std::partial_sum
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unsupported/Eigen/KroneckerProduct>
#include <vector>
#include "brains2/common/cone_color.hpp"
#include "brains2/external/expected.hpp"
#include "brains2/external/rapidcsv.hpp"

// Define a pair of matrices to hold the result
typedef std::pair<Eigen::MatrixXd, Eigen::MatrixXd> MatrixPair;

enum class SplineFittingError {
    PathShape,
    DeltaSLength,
    CoefficientsDimensions,
    SetGradient,
    SetConstraints,
    SetHessian,
    InitSolver,
    SolveQP
};

tl::expected<MatrixPair, SplineFittingError> fit_open_spline(const Eigen::MatrixXd& path,
                                                  double curv_weight = 1.0,
                                                  double initial_heading = M_PI / 2,
                                                  double final_heading = M_PI / 2,
                                                  bool verbose = false) {
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
    tripletListA.reserve(9*(N-1) + 3*(N-1) + 4);

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

    // Compute q = -B^T * path
    Eigen::VectorXd q = -B.transpose() * path;

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

    MatrixPair p_XY = std::make_pair(p_X, p_Y);

    return p_XY;
}

// Function to compute the lengths of each spline interval
tl::expected<Eigen::VectorXd, SplineFittingError> compute_spline_interval_lengths(
    const Eigen::MatrixXd& coeffs_X, const Eigen::MatrixXd& coeffs_Y, int no_interp_points = 100) {
    // Check that coeffs_X and coeffs_Y have the same dimensions and are Nx4 matrices
    if (coeffs_X.rows() != coeffs_Y.rows() || coeffs_X.cols() != 4 || coeffs_Y.cols() != 4) {
        return tl::make_unexpected(SplineFittingError::CoefficientsDimensions);
    }

    int N = coeffs_X.rows(); // Number of spline segments

    // Generate t_steps from 0.0 to 1.0 with no_interp_points points
    Eigen::VectorXd t_steps = Eigen::VectorXd::LinSpaced(no_interp_points, 0.0, 1.0);
    Eigen::VectorXd t_steps_square = t_steps.array().square();
    Eigen::VectorXd t_steps_cube = t_steps.array().cube();

    // Initialize delta_s vector to hold lengths of each spline interval
    Eigen::VectorXd delta_s(N);

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
        x.array() += coeff_X(0); // a0
        x.array() += coeff_X(1) * t_steps.array(); // a1 * t
        x.array() += coeff_X(2) * t_steps_square.array();  // a2 * t^2
        x.array() += coeff_X(3) * t_steps_cube.array();  // a3 * t^3

        y.array() += coeff_Y(0); // b0
        y.array() += coeff_Y(1) * t_steps.array(); // b1 * t
        y.array() += coeff_Y(2) * t_steps_square.array();  // b2 * t^2
        y.array() += coeff_Y(3) * t_steps_cube.array();  // b3 * t^3

        // Compute differences between consecutive points
        Eigen::VectorXd dx = x.segment(1, no_interp_points - 1) - x.segment(0, no_interp_points - 1);
        Eigen::VectorXd dy = y.segment(1, no_interp_points - 1) - y.segment(0, no_interp_points - 1);

        // Compute distances between consecutive points
        Eigen::VectorXd ds = (dx.array().square() + dy.array().square()).sqrt();

        // Sum distances to get the approximate length of the spline segment
        delta_s(i) = ds.sum();
    }

    return delta_s;
}

// Function to uniformly sample the spline
tl::expected<
    std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXi, Eigen::VectorXd, Eigen::VectorXd>,
    SplineFittingError>
uniformly_sample_spline(const Eigen::MatrixXd& coeffs_X,
                        const Eigen::MatrixXd& coeffs_Y,
                        const Eigen::VectorXd& delta_s,
                        int n_samples) {
    // Check that coeffs_X and coeffs_Y have shape (N, 4)
    int N = coeffs_X.rows();
    if (coeffs_X.rows() != coeffs_Y.rows() || coeffs_X.cols() != 4 || coeffs_Y.cols() != 4) {
        return tl::make_unexpected(SplineFittingError::CoefficientsDimensions);
    }
    if (delta_s.size() != N) {
        return tl::make_unexpected(SplineFittingError::DeltaSLength);
    }

    // Compute cumulative sum of delta_s
    Eigen::VectorXd s(N);
    std::partial_sum(delta_s.data(), delta_s.data() + N, s.data());

    // Generate s_interp: n_samples equally spaced values from 0 to s(N - 1), excluding endpoint
    Eigen::VectorXd s_interp(n_samples);
    double s_max = s(N - 1);
    double delta = s_max / n_samples;
    for (int i = 0; i < n_samples; ++i) {
        s_interp(i) = i * delta;
    }

    // Find idx_interp: index of the spline segment that contains each s_interp[i]
    Eigen::VectorXi idx_interp(n_samples);
    for (int i = 0; i < n_samples; ++i) {
        // Find the smallest index idx such that s_interp[i] < s[idx]
        auto it = std::upper_bound(s.data(), s.data() + N, s_interp(i));
        idx_interp(i) = static_cast<int>(it - s.data());
    }

    // Compute t_interp: parameter t within each spline segment
    Eigen::VectorXd t_interp(n_samples);
    for (int i = 0; i < n_samples; ++i) {
        int idx = idx_interp(i);
        if (idx > 0) {
            t_interp(i) = (s_interp(i) - s(idx - 1)) / delta_s(idx);
        }
        else {
            t_interp(i) = s_interp(i) / delta_s(0);
        }
    }

    // Evaluate X_interp and Y_interp using the spline coefficients
    Eigen::VectorXd X_interp(n_samples);
    Eigen::VectorXd Y_interp(n_samples);
    for (int i = 0; i < n_samples; ++i) {
        int idx = idx_interp(i);

        // Ensure idx is within bounds
        if (idx >= N)
            idx = N - 1;

        double t = t_interp(i);
        double t2 = t * t;
        double t3 = t2 * t;

        X_interp(i) = coeffs_X(idx, 0) + coeffs_X(idx, 1) * t + coeffs_X(idx, 2) * t2 + coeffs_X(idx, 3) * t3;
        Y_interp(i) = coeffs_Y(idx, 0) + coeffs_Y(idx, 1) * t + coeffs_Y(idx, 2) * t2 + coeffs_Y(idx, 3) * t3;
    }

    return std::make_tuple(X_interp, Y_interp, idx_interp, t_interp, s_interp);
}

int main() {
    // rapidcsv::Document cones("src/brains2/src/estimation/test_cones.csv");

    // const std::string path_input = "src/brains2/src/estimation/alpha_cones.csv";
    const std::string path_input = "src/brains2/src/estimation/test_cones.csv";
    // const std::string path_output = "src/brains2/src/estimation/interpolated_spline_alpha.csv";
    const std::string path_output = "src/brains2/src/estimation/interpolated_spline.csv";
    const size_t resample_points = 100;
    const double curv_weight = 1.0;

    rapidcsv::Document cones(path_input);

    Eigen::MatrixXd blue_cones;
    Eigen::MatrixXd yellow_cones;

    std::vector<double> cones_x = cones.GetColumn<double>("X");
    std::vector<double> cones_y = cones.GetColumn<double>("Y");
    std::vector<std::string> cones_type = cones.GetColumn<std::string>("color");

    size_t n_blue = 0, n_yellow = 0;

    for (size_t i = 0; i < cones_type.size(); ++i) {
        if (cones_type[i] == "blue") {
            ++n_blue;
        } else if (cones_type[i] == "yellow") {
            ++n_yellow;
        }
    }

    blue_cones.resize(n_blue, 2);
    yellow_cones.resize(n_yellow, 2);
    n_blue = 0;
    n_yellow = 0;

    for (size_t i = 0; i < cones_type.size(); ++i) {
        if (cones_type[i] == "blue") {
            blue_cones(n_blue, 0) = cones_x[i];
            blue_cones(n_blue, 1) = cones_y[i];
            ++n_blue;

        } else if (cones_type[i] == "yellow") {
            yellow_cones(n_yellow, 0) = cones_x[i];
            yellow_cones(n_yellow, 1) = cones_y[i];
            ++n_yellow;
        }
    }

    // print the cones
    // std::cout << "Blue cones (X, Y):\n";
    // for (size_t i = 0; i < blue_cones.rows(); ++i) {
    //     std::cout << "(" << blue_cones(i, 0) << ", " << blue_cones(i, 1) << ")\n";
    // }

    // std::cout << "\nYellow cones (X, Y):\n";
    // for (size_t i = 0; i < yellow_cones.rows(); ++i) {
    //     std::cout << "(" << yellow_cones(i, 0) << ", " << yellow_cones(i, 1) << ")\n";
    // }

    // For demonstration, we'll create a path by averaging the positions of blue and yellow cones.
    // This assumes that cones are placed in pairs along the track.

    auto start = std::chrono::high_resolution_clock::now();

    double initial_heading_yellow = atan2(yellow_cones(1, 1) - yellow_cones(0, 1),
                                          yellow_cones(1, 0) - yellow_cones(0, 0));
    double initial_heading_blue = atan2(blue_cones(1 , 1) - blue_cones(0, 1),
                                        blue_cones(1, 0) - blue_cones(0, 0));
    double final_heading_yellow = atan2(yellow_cones(yellow_cones.rows() - 1, 1) - yellow_cones(yellow_cones.rows() - 2, 1),
                                        yellow_cones(yellow_cones.rows() - 1, 0) - yellow_cones(yellow_cones.rows() - 2, 0));
    double final_heading_blue = atan2(blue_cones(blue_cones.rows() - 1, 1) - blue_cones(blue_cones.rows() - 2, 1),
                                      blue_cones(blue_cones.rows() - 1, 0) - blue_cones(blue_cones.rows() - 2, 0));

    MatrixPair splines_blue =
        fit_open_spline(blue_cones, curv_weight, initial_heading_blue, final_heading_blue).value();
    MatrixPair splines_yellow =
        fit_open_spline(yellow_cones, curv_weight, initial_heading_yellow, final_heading_yellow)
            .value();

    Eigen::VectorXd delta_s_blue =
        compute_spline_interval_lengths(splines_blue.first, splines_blue.second).value();
    Eigen::VectorXd delta_s_yellow =
        compute_spline_interval_lengths(splines_yellow.first, splines_yellow.second).value();

    std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXi, Eigen::VectorXd, Eigen::VectorXd>
        result_blue = uniformly_sample_spline(splines_blue.first,
                                              splines_blue.second,
                                              delta_s_blue,
                                              resample_points)
                          .value();

    std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXi, Eigen::VectorXd, Eigen::VectorXd>
        result_yellow = uniformly_sample_spline(splines_yellow.first,
                                                splines_yellow.second,
                                                delta_s_yellow,
                                                resample_points)
                            .value();

    auto [X_interp_blue, Y_interp_blue, idx_interp_blue, t_interp_blue, s_interp_blue] =
        result_blue;

    auto [X_interp_yellow, Y_interp_yellow, idx_interp_yellow, t_interp_yellow, s_interp_yellow] =
        result_yellow;

    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Elapsed time: " << elapsed_seconds.count() << "s\n";

    // print x_interp and y_interp to csv color, x, y
    std::ofstream file;
    file.open(path_output);
    file << "color,X,Y\n";
    for (size_t i = 0; i < X_interp_blue.size(); ++i) {
        file << "blue," << X_interp_blue(i) << "," << Y_interp_blue(i) << "\n";
    }
    for (size_t i = 0; i < X_interp_yellow.size(); ++i) {
        file << "yellow," << X_interp_yellow(i) << "," << Y_interp_yellow(i) << "\n";
    }

    return 0;
}