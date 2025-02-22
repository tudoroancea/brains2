
#include "brains2/estimation/center_line_estimation.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <limits>
#include <set>
#include "brains2/common/spline_fitting.hpp"
#include "brains2/external/expected.hpp"
#include "brains2/external/icecream.hpp"

namespace brains2::track_estimation {

tl::expected<double, CenterLineEstimationError> compute_shortest_distance(
    const Eigen::Vector2d& center_point,
    const Eigen::Vector2d& normal,
    const Eigen::Matrix2Xd& candidates) {
    if (candidates.cols() == 0) {
        return tl::make_unexpected(CenterLineEstimationError::EMPTY_INPUT);
    }

    // Compute segments between consecutive candidates (not directly used in distance
    // computation here but provided for consistency with the original structure)
    Eigen::MatrixX2d candidates_segments(candidates.cols() - 1, 2);
    for (int i = 0; i < candidates.cols() - 1; ++i) {
        candidates_segments.row(i) = (candidates.col(i + 1) - candidates.col(i)).transpose();
    }

    double min_dist = std::numeric_limits<double>::max();
    constexpr double epsilon = 1e-8;

    // Iterate over candidate segments
    for (int i = 0; i < candidates.cols() - 1; ++i) {
        Eigen::Vector2d p1 = candidates.col(i);
        Eigen::Vector2d p2 = candidates.col(i + 1);

        Eigen::Vector2d segment = p2 - p1;

        // Compute denominator = normal x segment using the 2D cross product
        double denom = normal.x() * segment.y() - normal.y() * segment.x();

        // If the lines are almost parallel, skip this candidate segment.
        if (std::fabs(denom) < epsilon) {
            continue;
        }

        // Compute the numerator:
        // (p1 - center_point) x segment
        double numer =
            (p1 - center_point).x() * segment.y() - (p1 - center_point).y() * segment.x();

        // Intersection parameter along the ray:
        double t = numer / denom;

        // Compute the distance along the ray.
        double dist = std::fabs(t);

        if (dist < min_dist) {
            min_dist = dist;
        }
    }

    return min_dist;
}

tl::expected<CenterLine, CenterLineEstimationError> compute_center_line(
    const Eigen::VectorXd& X_blue,
    const Eigen::VectorXd& Y_blue,
    const Eigen::VectorXd& X_yellow,
    const Eigen::VectorXd& Y_yellow,
    const double curv_weight,
    const int no_interp_points,
    const int neighborhood_search_track_width,
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

    VectorPair track_width = {Eigen::VectorXd::Zero(spline_interp.X.size()),
                              Eigen::VectorXd::Zero(spline_interp.X.size())};

    // for (size_t i = 0; i < X_blue.size(); ++i) {
    //     // Compute differences for the blue track relative to the centerline.
    //     Eigen::VectorXd dx_blue = X_blue - spline_interp.X;
    //     Eigen::VectorXd dy_blue = Y_blue - spline_interp.Y;

    //     // Compute differences for the yellow track relative to the centerline.
    //     Eigen::VectorXd dx_yellow = X_yellow - spline_interp.X;
    //     Eigen::VectorXd dy_yellow = Y_yellow - spline_interp.Y;

    //     // Compute track widths
    //     track_width.first = ((dx_blue.array().square() +
    //     dy_blue.array().square()).sqrt()).matrix(); track_width.second =
    //         ((dx_yellow.array().square() + dy_yellow.array().square()).sqrt()).matrix();
    // }

    // CenterLine center_line_out({{spline_interp.X, spline_interp.Y}, track_width});

    // return center_line_out;

    Eigen::Matrix2Xd closest_neighbors_index =
        Eigen::Matrix2Xd::Constant(2, spline_interp.X.size(), std::numeric_limits<double>::max());
    // Loop over each point in the spline.
    for (long i = 0; i < spline_interp.X.size(); ++i) {
        double min_squared_distance_blue = std::numeric_limits<double>::max();
        double min_squared_distance_yellow = std::numeric_limits<double>::max();
        // Ensure j goes from (i - neighborhood_search_track_width) to (i +
        // neighborhood_search_track_width)
        for (long j = i - neighborhood_search_track_width; j <= i + neighborhood_search_track_width;
             ++j) {
            // Check valid index range.
            if (j < 0 || j >= spline_interp.X.size()) {
                continue;
            }

            // Compute the squared Euclidean distance between points i and j.
            double dxb = spline_interp.X[i] - X_blue[j];
            double dyb = spline_interp.Y[i] - Y_blue[j];
            double squared_distance_blue = dxb * dxb + dyb * dyb;
            double dxy = spline_interp.X[i] - X_yellow[j];
            double dyy = spline_interp.Y[i] - Y_yellow[j];
            double squared_distance_yellow = dxy * dxy + dyy * dyy;

            // If this distance is less than the current minimum, update the minimum and store
            // the coordinate of the new closest neighbor.
            if (squared_distance_blue < min_squared_distance_blue) {
                min_squared_distance_blue = squared_distance_blue;
                // save only the value of j of index 1
                closest_neighbors_index(0, i) = j;
            }
            if (squared_distance_yellow < min_squared_distance_yellow) {
                min_squared_distance_yellow = squared_distance_yellow;
                // save only the value of j of index 2
                closest_neighbors_index(1, i) = j;
            }
        }
        // IC(closest_neighbors_index.col(i));
        Eigen::Vector2d center_point = Eigen::Vector2d(spline_interp.X[i], spline_interp.Y[i]);
        // IC(center_point);
    }

    auto excpected_center_line_heading =
        center_line_fitter.get_heading(spline_interp.idx, spline_interp.t);

    if (!excpected_center_line_heading) {
        return tl::make_unexpected(CenterLineEstimationError::SPLINE_FITTING_ERROR);
    }

    Eigen::VectorXd center_line_heading = excpected_center_line_heading.value();

    for (long i = 0; i < spline_interp.X.size(); ++i) {
        // Compute the normal vector to the center line.
        Eigen::Vector2d normal(-std::sin(center_line_heading[i]), std::cos(center_line_heading[i]));

        Eigen::Matrix2Xd candidates;

        // blue distance
        if (closest_neighbors_index(0, i) == 0) {
            candidates = Eigen::MatrixX2d{
                {X_blue[closest_neighbors_index(0, i)], X_blue[closest_neighbors_index(0, i) + 1]},
                {Y_blue[closest_neighbors_index(0, i)], Y_blue[closest_neighbors_index(0, i) + 1]}};
        } else if (closest_neighbors_index(0, i) == spline_interp.X.size() - 1) {
            candidates = Eigen::MatrixX2d{
                {X_blue[closest_neighbors_index(0, i) - 1], X_blue[closest_neighbors_index(0, i)]},
                {Y_blue[closest_neighbors_index(0, i) - 1], Y_blue[closest_neighbors_index(0, i)]}};
        } else {
            candidates = Eigen::MatrixX2d{{X_blue[closest_neighbors_index(0, i) - 1],
                                           X_blue[closest_neighbors_index(0, i)],
                                           X_blue[closest_neighbors_index(0, i) + 1]},
                                          {Y_blue[closest_neighbors_index(0, i) - 1],
                                           Y_blue[closest_neighbors_index(0, i)],
                                           Y_blue[closest_neighbors_index(0, i) + 1]}};
        }

        Eigen::Vector2d center_point(spline_interp.X[i], spline_interp.Y[i]);

        auto expected_distance = compute_shortest_distance(center_point, normal, candidates);
        if (!expected_distance) {
            return tl::make_unexpected(CenterLineEstimationError::TRACK_WIDTH_ERROR);
        }
        track_width.first(i) = expected_distance.value();

        // yellow distance
        if (closest_neighbors_index(1, i) == 0) {
            candidates = Eigen::MatrixX2d{{X_yellow[closest_neighbors_index(1, i)],
                                           X_yellow[closest_neighbors_index(1, i) + 1]},
                                          {Y_yellow[closest_neighbors_index(1, i)],
                                           Y_yellow[closest_neighbors_index(1, i) + 1]}};
        } else if (closest_neighbors_index(1, i) == spline_interp.X.size() - 1) {
            candidates = Eigen::MatrixX2d{{X_yellow[closest_neighbors_index(1, i) - 1],
                                           X_yellow[closest_neighbors_index(1, i)]},
                                          {Y_yellow[closest_neighbors_index(1, i) - 1],
                                           Y_yellow[closest_neighbors_index(1, i)]}};
        } else {
            candidates = Eigen::MatrixX2d{{X_yellow[closest_neighbors_index(1, i) - 1],
                                           X_yellow[closest_neighbors_index(1, i)],
                                           X_yellow[closest_neighbors_index(1, i) + 1]},
                                          {Y_yellow[closest_neighbors_index(1, i) - 1],
                                           Y_yellow[closest_neighbors_index(1, i)],
                                           Y_yellow[closest_neighbors_index(1, i) + 1]}};
        }

        expected_distance = compute_shortest_distance(center_point, normal, candidates);
        if (!expected_distance) {
            return tl::make_unexpected(CenterLineEstimationError::TRACK_WIDTH_ERROR);
        }
        track_width.second(i) = expected_distance.value();
    }

    CenterLine center_line({{spline_interp.X, spline_interp.Y}, track_width});

    return center_line;
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
