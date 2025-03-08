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


#include "brains2/estimation/center_line_estimation.hpp"
#include <set>
#include "brains2/common/spline_fitting.hpp"

namespace brains2::track_estimation {

tl::expected<VectorPair, CenterLineEstimationError> compute_center_line(
    const Eigen::VectorXd& X_blue,
    const Eigen::VectorXd& Y_blue,
    const Eigen::VectorXd& X_yellow,
    const Eigen::VectorXd& Y_yellow,
    const double curv_weight,
    const int no_interp_points,
    const bool verbose) {
    // Check if any of the input vectors are empty
    if (X_blue.size() == 0 || Y_blue.size() == 0 || X_yellow.size() == 0 || Y_yellow.size() == 0) {
        return tl::make_unexpected(CenterLineEstimationError::EMPTY_INPUT);
    }
    // Check that sizes of X and Y are equal for each boundary
    if (X_blue.size() != Y_blue.size() || X_yellow.size() != Y_yellow.size()) {
        return tl::make_unexpected(CenterLineEstimationError::SIZE_MISMATCH);
    }

    using IndexPair = std::pair<int, int>;
    std::set<IndexPair> matched_pairs;

    // Find closest points from blue to yellow boundary
    for (int i = 0; i < X_blue.size(); ++i) {
        Eigen::Vector2d blue_point(X_blue[i], Y_blue[i]);

        // See:  (blue_point - yellow_points).rowwise().squaredNorm().minCoeff(&min_index);
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
    // See: blue_cones.tail(blue_cones.rows()-1) -
    // blue_cones.head(blue_cones.rows()-1)).rowwis().norm();
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
        // return tl::make_unexpected(expected_center_line.error());
        return tl::make_unexpected(CenterLineEstimationError::SPLINE_FITTING_ERROR);
    }

    SplineFitter center_line_fitter = expected_center_line.value();
    auto fit_result = center_line_fitter.fit_open_spline();

    if (!fit_result) {
        // return tl::make_unexpected(fit_result.error());
        return tl::make_unexpected(CenterLineEstimationError::SPLINE_FITTING_ERROR);
    }

    auto length_result = center_line_fitter.compute_spline_interval_lengths();

    if (!length_result) {
        // return tl::make_unexpected(length_result.error());
        return tl::make_unexpected(CenterLineEstimationError::SPLINE_FITTING_ERROR);
    }

    auto sample_result = center_line_fitter.uniformly_sample_spline(no_interp_points);

    if (!sample_result) {
        // return tl::make_unexpected(sample_result.error());
        return tl::make_unexpected(CenterLineEstimationError::SPLINE_FITTING_ERROR);
    }

    SplineParametrization spline_interp = sample_result.value();

    return std::make_pair(spline_interp.X, spline_interp.Y);
}

// tl::expected<VectorPair, CenterLineEstimationError> compute_center_line(
//     const Eigen::VectorXd& X_blue,
//     const Eigen::VectorXd& Y_blue,
//     const Eigen::VectorXd& X_yellow,
//     const Eigen::VectorXd& Y_yellow) {

//     long sizes[4] = {X_blue.size(), Y_blue.size(), X_yellow.size(), Y_yellow.size()};
//     // check if all sizes are equal
//     if (not std::all_of(std::begin(sizes), std::end(sizes), [&](long size) {
//             return size > 0;
//         })) {
//         return tl::make_unexpected(CenterLineEstimationError::EMPTY_INPUT);
//     }

//     if (not std::all_of(std::begin(sizes), std::end(sizes), [&](long size) {
//             return size == sizes[0];
//         })) {
//         return tl::make_unexpected(CenterLineEstimationError::SIZE_MISMATCH);
//     }

//     Eigen::VectorXd X_center = (X_blue + X_yellow) / 2;
//     Eigen::VectorXd Y_center = (Y_blue + Y_yellow) / 2;

//     return std::make_pair(X_center, Y_center);
// }

}  // namespace brains2::track_estimation