// SplineFitter.hpp

#ifndef SPLINE_FITTER_HPP
#define SPLINE_FITTER_HPP

#include <osqp/osqp.h>
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <tuple>
#include <unsupported/Eigen/KroneckerProduct>
#include "brains2/external/expected.hpp"

// Define a pair of matrices to hold the result
typedef std::pair<Eigen::MatrixXd, Eigen::MatrixXd> MatrixPair;

enum class SplineFittingError {
    PATH_SHAPE,
    DELTA_S_LENGTH,
    COEFFICIENTS_DIMENSIONS,
    SET_GRADIENT,
    SET_CONSTRAINTS,
    SET_HESSIAN,
    INIT_SOLVER,
    SOLVE_QP,
    NOT_ENOUGH_POINTS
};

class SplineFitter {
public:
    // Constructor
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
            default:
                return "UNKNOWN_ERROR";
        }
    }

    // Method to fit the open spline
    tl::expected<void, SplineFittingError> fit_open_spline();

    // Method to compute the interval lengths
    tl::expected<void, SplineFittingError> compute_spline_interval_lengths(
        int no_interp_points = 100);

    // Method to uniformly sample the spline
    tl::expected<std::tuple<Eigen::VectorXd,
                            Eigen::VectorXd,
                            Eigen::VectorXi,
                            Eigen::VectorXd,
                            Eigen::VectorXd>,
                 SplineFittingError>
    uniformly_sample_spline(int n_samples);

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

#endif  // SPLINE_FITTER_HPP