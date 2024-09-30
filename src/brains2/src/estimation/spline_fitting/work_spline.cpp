// main.cpp

#include "brains2/estimation/spline_fitting/spline_fitter.h"

int main() {
    // Read cone data from CSV file
    // ... (Your code to read and parse the CSV file)

    // Assume you have populated blue_cones and yellow_cones Eigen matrices
    Eigen::MatrixXd blue_cones;    // Replace with actual data
    Eigen::MatrixXd yellow_cones;  // Replace with actual data

    const size_t resample_points = 100;
    const double curv_weight = 1.0;

    // For demonstration, we'll use hardcoded initial and final headings
    double initial_heading_blue = M_PI / 2;
    double final_heading_blue = M_PI / 2;

    double initial_heading_yellow = M_PI / 2;
    double final_heading_yellow = M_PI / 2;

    // Blue cones
    SplineFitter blue_spline_fitter(blue_cones,
                                    curv_weight,
                                    initial_heading_blue,
                                    final_heading_blue,
                                    false);
    auto fit_result_blue = blue_spline_fitter.fit_open_spline();
    if (!fit_result_blue) {
        std::cerr << "Error fitting spline for blue cones: "
                  << static_cast<int>(fit_result_blue.error()) << "\n";
        return 1;
    }

    auto length_result_blue = blue_spline_fitter.compute_spline_interval_lengths();
    if (!length_result_blue) {
        std::cerr << "Error computing spline interval lengths for blue cones: "
                  << static_cast<int>(length_result_blue.error()) << "\n";
        return 1;
    }

    auto sample_result_blue = blue_spline_fitter.uniformly_sample_spline(resample_points);
    if (!sample_result_blue) {
        std::cerr << "Error sampling spline for blue cones: "
                  << static_cast<int>(sample_result_blue.error()) << "\n";
        return 1;
    }

    auto [X_interp_blue, Y_interp_blue, idx_interp_blue, t_interp_blue, s_interp_blue] =
        sample_result_blue.value();

    // Yellow cones
    SplineFitter yellow_spline_fitter(yellow_cones,
                                      curv_weight,
                                      initial_heading_yellow,
                                      final_heading_yellow,
                                      false);
    auto fit_result_yellow = yellow_spline_fitter.fit_open_spline();
    if (!fit_result_yellow) {
        std::cerr << "Error fitting spline for yellow cones: "
                  << static_cast<int>(fit_result_yellow.error()) << "\n";
        return 1;
    }

    auto length_result_yellow = yellow_spline_fitter.compute_spline_interval_lengths();
    if (!length_result_yellow) {
        std::cerr << "Error computing spline interval lengths for yellow cones: "
                  << static_cast<int>(length_result_yellow.error()) << "\n";
        return 1;
    }

    auto sample_result_yellow = yellow_spline_fitter.uniformly_sample_spline(resample_points);
    if (!sample_result_yellow) {
        std::cerr << "Error sampling spline for yellow cones: "
                  << static_cast<int>(sample_result_yellow.error()) << "\n";
        return 1;
    }

    auto [X_interp_yellow, Y_interp_yellow, idx_interp_yellow, t_interp_yellow, s_interp_yellow] =
        sample_result_yellow.value();

    // Output the interpolated points to CSV or visualize them
    // For example, write to CSV:
    std::ofstream file("interpolated_spline.csv");
    file << "color,X,Y\n";
    for (int i = 0; i < X_interp_blue.size(); ++i) {
        file << "blue," << X_interp_blue(i) << "," << Y_interp_blue(i) << "\n";
    }
    for (int i = 0; i < X_interp_yellow.size(); ++i) {
        file << "yellow," << X_interp_yellow(i) << "," << Y_interp_yellow(i) << "\n";
    }
    file.close();

    return 0;
}