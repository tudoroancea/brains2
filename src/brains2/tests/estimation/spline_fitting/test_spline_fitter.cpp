// SplineFitterTest.cpp

#include <gtest/gtest.h>
#include "brains2/estimation/spline_fitting/spline_fitter.h"

TEST(SplineFitterTest, EmptyPath) {
    Eigen::MatrixXd empty_path;
    SplineFitter spline_fitter(empty_path);
    auto result = spline_fitter.fit_open_spline();
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), SplineFittingError::PathShape);
}

TEST(SplineFitterTest, SinglePointPath) {
    Eigen::MatrixXd single_point(1, 2);
    single_point << 0.0, 0.0;
    SplineFitter spline_fitter(single_point);
    auto result = spline_fitter.fit_open_spline();
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), SplineFittingError::PathShape);
}

TEST(SplineFitterTest, StraightLine) {
    Eigen::MatrixXd path(4, 2);
    path << 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0;
    SplineFitter spline_fitter(path);
    auto fit_result = spline_fitter.fit_open_spline();
    ASSERT_TRUE(fit_result.has_value());

    auto length_result = spline_fitter.compute_spline_interval_lengths();
    ASSERT_TRUE(length_result.has_value());

    int n_samples = 10;
    auto sample_result = spline_fitter.uniformly_sample_spline(n_samples);
    ASSERT_TRUE(sample_result.has_value());

    auto [X_interp, Y_interp, idx_interp, t_interp, s_interp] = sample_result.value();

    // Check that the sampled points lie along y = x
    for (int i = 0; i < X_interp.size(); ++i) {
        EXPECT_NEAR(Y_interp(i), X_interp(i), 1e-6);
    }
}

TEST(SplineFitterTest, IdenticalPoints) {
    Eigen::MatrixXd path(4, 2);
    path << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    SplineFitter spline_fitter(path);
    auto fit_result = spline_fitter.fit_open_spline();
    EXPECT_FALSE(fit_result.has_value());
    // Depending on the implementation, the specific error may vary
    // For this case, we might expect an error related to solver initialization or path shape
}

TEST(SplineFitterTest, CircularArc) {
    // Approximate a quarter circle arc from (1,0) to (0,1)
    Eigen::MatrixXd path(5, 2);
    path << 1.0, 0.0, std::sqrt(2) / 2, std::sqrt(2) / 2, 0.0, 1.0, -std::sqrt(2) / 2,
        std::sqrt(2) / 2, -1.0, 0.0;
    SplineFitter spline_fitter(path);
    auto fit_result = spline_fitter.fit_open_spline();
    ASSERT_TRUE(fit_result.has_value());

    auto length_result = spline_fitter.compute_spline_interval_lengths();
    ASSERT_TRUE(length_result.has_value());

    int n_samples = 20;
    auto sample_result = spline_fitter.uniformly_sample_spline(n_samples);
    ASSERT_TRUE(sample_result.has_value());

    auto [X_interp, Y_interp, idx_interp, t_interp, s_interp] = sample_result.value();

    // Check that the points lie approximately on the unit circle
    for (int i = 0; i < X_interp.size(); ++i) {
        double radius = std::sqrt(X_interp(i) * X_interp(i) + Y_interp(i) * Y_interp(i));
        EXPECT_NEAR(radius, 1.0, 1e-2);
    }
}