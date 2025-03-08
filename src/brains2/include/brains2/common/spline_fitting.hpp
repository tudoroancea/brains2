// Copyright 2025 Tudor Oancea, Mateo Berthet
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef SPLINE_FITTING_HPP
#define SPLINE_FITTING_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <unsupported/Eigen/KroneckerProduct>
#include "brains2/external/expected.hpp"

typedef std::pair<Eigen::MatrixXd, Eigen::MatrixXd> MatrixPair;
typedef std::pair<Eigen::VectorXd, Eigen::VectorXd> VectorPair;

namespace brains2::track_estimation {

enum class SplineFittingError {
    PATH_SHAPE,
    DELTA_S_LENGTH,
    COEFFICIENTS_DIMENSIONS,
    SET_GRADIENT,
    SET_CONSTRAINTS,
    SET_HESSIAN,
    INIT_SOLVER,
    SOLVE_QP,
    NOT_ENOUGH_POINTS,
    SIZE_MISMATCH,
    INDEX_OUT_OF_BOUNDS,
    ZERO_DENOMINATOR,
    EMPTY_INPUT
};

/*
 * @brief Struct to store the parametrization of a spline
 *        X: x-coordinates of the spline
 *        Y: y-coordinates of the spline
 *        idx: indices of the spline coefficients
 *        t: continuous parameter along the spline (0 <= t <= 1)
 *        s: cumulative arc length
 */
struct SplineParametrization {
    Eigen::VectorXd X;
    Eigen::VectorXd Y;
    Eigen::VectorXi idx;
    Eigen::VectorXd t;
    Eigen::VectorXd s;
};

/*
 * @brief Class to fit a cubic spline to a given path
 *        The path is given as a matrix with shape (N, 2) where N is the number of points
 *        Can get the heading of the tangent vector and the curvature of the spline
 * @params: path: matrix with shape (N, 2) where N is the number of points
 *          curv_weight: weight for the curvature term in the optimization problem
 *          verbose: flag to print debug information
 */

class SplineFitter {
public:
    // Constructor
    SplineFitter() = delete;
    static tl::expected<SplineFitter, SplineFittingError> create(const Eigen::MatrixXd& path,
                                                                 double curv_weight = 1.0,
                                                                 bool verbose = false);

    static inline std::string to_string(SplineFittingError error) {
        switch (error) {
            case SplineFittingError::PATH_SHAPE:
                return "PATH_SHAPE";
            case SplineFittingError::DELTA_S_LENGTH:
                return "DELTA_S_LENGTH";
            case SplineFittingError::COEFFICIENTS_DIMENSIONS:
                return "COEFFICIENTS_DIMENSIONS";
            case SplineFittingError::SET_GRADIENT:
                return "SET_GRADIENT";
            case SplineFittingError::SET_CONSTRAINTS:
                return "SET_CONSTRAINTS";
            case SplineFittingError::SET_HESSIAN:
                return "SET_HESSIAN";
            case SplineFittingError::INIT_SOLVER:
                return "INIT_SOLVER";
            case SplineFittingError::SOLVE_QP:
                return "SOLVE_QP";
            case SplineFittingError::NOT_ENOUGH_POINTS:
                return "NOT_ENOUGH_POINTS";
            case SplineFittingError::SIZE_MISMATCH:
                return "SIZE_MISMATCH";
            case SplineFittingError::INDEX_OUT_OF_BOUNDS:
                return "INDEX_OUT_OF_BOUNDS";
            case SplineFittingError::ZERO_DENOMINATOR:
                return "ZERO_DENOMINATOR";
            case SplineFittingError::EMPTY_INPUT:
                return "EMPTY_INPUT";
            default:
                return "UNKNOWN_ERROR";
        }
    }

    tl::expected<Eigen::VectorXd, SplineFittingError> get_heading(const Eigen::VectorXi& idx_interp,
                                                                  const Eigen::VectorXd& t_interp);

    tl::expected<Eigen::VectorXd, SplineFittingError> get_curvature(
        const Eigen::VectorXi& idx_interp, const Eigen::VectorXd& t_interp);

    // Method to fit the open spline
    tl::expected<void, SplineFittingError> fit_open_spline();

    // Method to compute the interval lengths
    tl::expected<void, SplineFittingError> compute_spline_interval_lengths(
        int no_interp_points = 100);

    // Method to uniformly sample the spline
    tl::expected<SplineParametrization, SplineFittingError> uniformly_sample_spline(int n_samples);

    // Getters for the coefficients and delta_s
    const MatrixPair& get_spline_coefs() const;
    const Eigen::VectorXd& get_delta_s() const;

private:
    Eigen::MatrixXd path;
    double curv_weight;
    double initial_heading;
    double final_heading;
    bool verbose;

    MatrixPair spline_coefficients;
    Eigen::VectorXd delta_s;

    SplineFitter(const Eigen::MatrixXd& path, double curv_weight = 1.0, bool verbose = false);
};

}  // namespace brains2::track_estimation

#endif  // SPLINE_FITTING_HPP
