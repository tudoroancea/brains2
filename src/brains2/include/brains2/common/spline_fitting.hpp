// spline_fitter.hpp

#ifndef SPLINE_FITTER_HPP
#define SPLINE_FITTER_HPP

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

struct SplineParametrization {
    Eigen::VectorXd X;
    Eigen::VectorXd Y;
    Eigen::VectorXi idx;
    Eigen::VectorXd t;
    Eigen::VectorXd s;
};

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

#endif  // SPLINE_FITTER_HPP