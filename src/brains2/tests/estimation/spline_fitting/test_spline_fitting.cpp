#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include "brains2/common/spline_fitting.hpp"
#include "brains2/external/icecream.hpp"

using namespace brains2::track_estimation;

TEST(SplineFitterTest, EmptyPath) {
    Eigen::MatrixXd empty_path;
    auto result = SplineFitter::create(empty_path);
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), SplineFittingError::NOT_ENOUGH_POINTS);
}

TEST(SplineFitterTest, SinglePointPath) {
    Eigen::MatrixXd single_point(1, 2);
    single_point << 0.0, 0.0;
    auto result = SplineFitter::create(single_point);
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), SplineFittingError::NOT_ENOUGH_POINTS);
}

TEST(SplineFitterTest, StraightLine) {
    Eigen::MatrixXd path(4, 2);
    path << 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0;
    SplineFitter spline_fitter = SplineFitter::create(path).value();
    auto fit_result = spline_fitter.fit_open_spline();
    ASSERT_TRUE(fit_result.has_value());
    auto length_result = spline_fitter.compute_spline_interval_lengths();
    ASSERT_TRUE(length_result.has_value());
    int n_samples = 10;
    auto sample_result = spline_fitter.uniformly_sample_spline(n_samples);
    ASSERT_TRUE(sample_result.has_value());
    SplineParametrization spline_interp = sample_result.value();
    // Check that the sampled points lie along y = x
    for (int i = 0; i < spline_interp.X.size(); ++i) {
        EXPECT_NEAR(spline_interp.Y(i), spline_interp.X(i), 1e-3);
    }
}

TEST(SplineFitterTest, IdenticalPoints) {
    Eigen::MatrixXd path(4, 2);
    path << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    SplineFitter spline_fitter = SplineFitter::create(path, 0.1).value();
    auto fit_result = spline_fitter.fit_open_spline();
    EXPECT_FALSE(fit_result.has_value());
    // Depending on the implementation, the specific error may vary
    // For this case, we might expect an error related to solver initialization or path shape
    EXPECT_EQ(fit_result.error(), SplineFittingError::INIT_SOLVER);
}

TEST(SplineFitterTest, CircularArc) {
    // Approximate a quarter circle arc from (1,0) to (0,1)
    std::vector<double> angles =
        {0.0, M_PI / 6, M_PI / 3, M_PI / 2, 2 * M_PI / 3, 5 * M_PI / 6, M_PI};
    Eigen::MatrixXd path(angles.size(), 2);
    for (size_t i = 0; i < angles.size(); ++i) {
        path(i, 0) = std::cos(angles[i]);
        path(i, 1) = std::sin(angles[i]);
    }
    double real_radius = 2.5;
    path = path * real_radius;  // Scale the circle to radius 2
    SplineFitter spline_fitter = SplineFitter::create(path, 0.1).value();
    auto fit_result = spline_fitter.fit_open_spline();
    ASSERT_TRUE(fit_result.has_value());

    auto length_result = spline_fitter.compute_spline_interval_lengths();
    ASSERT_TRUE(length_result.has_value());

    int n_samples = 10;
    auto sample_result = spline_fitter.uniformly_sample_spline(n_samples);
    ASSERT_TRUE(sample_result.has_value());

    SplineParametrization spline_interp = sample_result.value();

    // Check that the points lie approximately on the unit circle
    for (int i = 0; i < spline_interp.X.size(); ++i) {
        double radius = std::sqrt(spline_interp.X(i) * spline_interp.X(i) +
                                  spline_interp.Y(i) * spline_interp.Y(i));
        EXPECT_NEAR(radius, real_radius, 1e-1);
    }
}