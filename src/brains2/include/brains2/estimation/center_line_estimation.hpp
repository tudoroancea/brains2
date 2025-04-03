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

#ifndef BRAINS2__ESTIMATION__CENTER_LINE_ESTIMATION_HPP_
#define BRAINS2__ESTIMATION__CENTER_LINE_ESTIMATION_HPP_

#include <string>
#include <utility>
#include "brains2/external/expected.hpp"
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Eigen/SparseCore"

typedef std::pair<Eigen::VectorXd, Eigen::VectorXd> VectorPair;

namespace brains2::track_estimation {

enum class CenterLineEstimationError { SIZE_MISMATCH, EMPTY_INPUT, SPLINE_FITTING_ERROR };

/*
 * @brief Function to compute the center line between two boundaries
 * @params X_blue: x-coordinates of the blue boundary
 *         Y_blue: y-coordinates of the blue boundary
 *         X_yellow: x-coordinates of the yellow boundary
 *         Y_yellow: y-coordinates of the yellow boundary
 *         curv_weight: weight for the curvature term in the optimization problem
 *         no_interp_points: number of points to interpolate for the center line after spline
 *         fitting
           verbose: flag to print debug information
 * @return A pair of vectors with the x and y coordinates of the center line
 */

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

#endif  // BRAINS2__ESTIMATION__CENTER_LINE_ESTIMATION_HPP_
