// main.cpp
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include "brains2/estimation/spline_fitting/spline_fitter.h"
#include "brains2/external/icecream.hpp"
#include "brains2/external/rapidcsv.hpp"

int main() {
    // const std::string path_input = "src/brains2/src/estimation/alpha_cones.csv";
    // const std::string path_input = "src/brains2/src/estimation/test_cones.csv";
    const std::string path_input = "src/brains2/src/estimation/circle_cones.csv";
    // const std::string path_output = "src/brains2/src/estimation/interpolated_spline_alpha.csv";
    // const std::string path_output = "src/brains2/src/estimation/interpolated_spline.csv";
    const std::string path_output = "src/brains2/src/estimation/circle_spline.csv";
    const size_t resample_points = 30;
    const double curv_weight = 0.1;

    rapidcsv::Document cones(path_input);

    Eigen::MatrixXd blue_cones;
    Eigen::MatrixXd yellow_cones;

    std::vector<double> cones_x = cones.GetColumn<double>("X");
    std::vector<double> cones_y = cones.GetColumn<double>("Y");
    std::vector<std::string> cones_type = cones.GetColumn<std::string>("color");

    size_t n_blue = 0, n_yellow = 0;

    for (size_t i = 0; i < cones_type.size(); ++i) {
        if (cones_type[i] == "blue") {
            ++n_blue;
        } else if (cones_type[i] == "yellow") {
            ++n_yellow;
        }
    }

    blue_cones.resize(n_blue, 2);
    yellow_cones.resize(n_yellow, 2);
    n_blue = 0;
    n_yellow = 0;

    for (size_t i = 0; i < cones_type.size(); ++i) {
        if (cones_type[i] == "blue") {
            blue_cones(n_blue, 0) = cones_x[i];
            blue_cones(n_blue, 1) = cones_y[i];
            ++n_blue;

        } else if (cones_type[i] == "yellow") {
            yellow_cones(n_yellow, 0) = cones_x[i];
            yellow_cones(n_yellow, 1) = cones_y[i];
            ++n_yellow;
        }
    }

    auto start = std::chrono::high_resolution_clock::now();

    // Blue cones
    SplineFitter blue_spline_fitter = SplineFitter::create(blue_cones, curv_weight).value();
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
    // SplineFitter yellow_spline_fitter = SplineFitter::create(yellow_cones, curv_weight).value();
    // auto fit_result_yellow = yellow_spline_fitter.fit_open_spline();
    // if (!fit_result_yellow) {
    //     std::cerr << "Error fitting spline for yellow cones: "
    //               << static_cast<int>(fit_result_yellow.error()) << "\n";
    //     return 1;
    // }

    // auto length_result_yellow = yellow_spline_fitter.compute_spline_interval_lengths();
    // if (!length_result_yellow) {
    //     std::cerr << "Error computing spline interval lengths for yellow cones: "
    //               << static_cast<int>(length_result_yellow.error()) << "\n";
    //     return 1;
    // }

    // auto sample_result_yellow = yellow_spline_fitter.uniformly_sample_spline(resample_points);
    // if (!sample_result_yellow) {
    //     std::cerr << "Error sampling spline for yellow cones: "
    //               << static_cast<int>(sample_result_yellow.error()) << "\n";
    //     return 1;
    // }

    // auto [X_interp_yellow, Y_interp_yellow, idx_interp_yellow, t_interp_yellow, s_interp_yellow]
    // =
    //     sample_result_yellow.value();

    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Elapsed time: " << elapsed_seconds.count() << "s\n";

    // Output the interpolated points to CSV or visualize them
    // For example, write to CSV:
    std::ofstream file(path_output);
    file << "color,X,Y\n";
    for (int i = 0; i < X_interp_blue.size(); ++i) {
        file << "blue," << X_interp_blue(i) << "," << Y_interp_blue(i) << "\n";
    }
    // for (int i = 0; i < X_interp_yellow.size(); ++i) {
    //     file << "yellow," << X_interp_yellow(i) << "," << Y_interp_yellow(i) << "\n";
    // }
    file.close();

    return 0;
}