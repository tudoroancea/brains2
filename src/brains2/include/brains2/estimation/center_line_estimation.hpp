// center_line_estimation.hpp

#ifndef CENTER_LINE_ESTIMATION_HPP
#define CENTER_LINE_ESTIMATION_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include "brains2/external/expected.hpp"

typedef std::pair<Eigen::VectorXd, Eigen::VectorXd> VectorPair;

namespace brains2::track_estimation {

enum class CenterLineEstimationError {
    SIZE_MISMATCH,
    EMPTY_INPUT,
    SPLINE_FITTING_ERROR
};

tl::expected<VectorPair, CenterLineEstimationError> compute_center_line(
    const Eigen::VectorXd& X_blue,
    const Eigen::VectorXd& Y_blue,
    const Eigen::VectorXd& X_yellow,
    const Eigen::VectorXd& Y_yellow,
    const double curv_weight = 1.0,
    const int no_interp_points = 100,
    const bool verbose = false);

inline std::string to_string(CenterLineEstimationError error) {
    switch (error) {
        case CenterLineEstimationError::SIZE_MISMATCH:
            return "SIZE_MISMATCH";
        case CenterLineEstimationError::EMPTY_INPUT:
            return "EMPTY_INPUT";
        case CenterLineEstimationError::SPLINE_FITTING_ERROR:
            return "SPLINE_FITTING_ERROR";
        default:
            return "UNKNOWN_ERROR";
    }
}

}  // namespace brains2::track_estimation

#endif  // CENTER_LINE_ESTIMATION_HPP