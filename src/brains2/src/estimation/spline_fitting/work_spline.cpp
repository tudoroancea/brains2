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

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include "brains2/common/spline_fitting.hpp"
#include "brains2/estimation/center_line_estimation.hpp"
#include "brains2/external/icecream.hpp"
#include "brains2/external/rapidcsv.hpp"
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Eigen/SparseCore"

using brains2::track_estimation::compute_center_line;
using brains2::track_estimation::SplineFitter;
using brains2::track_estimation::SplineParametrization;

int main() {
    const std::string path_input = "src/brains2/src/estimation/alpha_cones.csv";
    // const std::string path_input = "src/brains2/src/estimation/test_cones.csv";
    // const std::string path_input = "src/brains2/src/estimation/circle_cones.csv";
    const std::string path_output = "src/brains2/src/estimation/interpolated_spline_alpha.csv";
    // const std::string path_output = "src/brains2/src/estimation/interpolated_spline.csv";
    // const std::string path_output = "src/brains2/src/estimation/circle_spline.csv";
    const size_t resample_points = 100;
    const double curv_weight = 0.1;
    const bool verbose = false;

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
    auto expected_blue_spline_fitter = SplineFitter::create(blue_cones, curv_weight);
    if (!expected_blue_spline_fitter) {
        std::cerr << "Error creating SplineFitter for yellow cones: "
                  << SplineFitter::to_string(expected_blue_spline_fitter.error()) << "\n";
        return 1;
    }

    SplineFitter blue_spline_fitter = expected_blue_spline_fitter.value();
    auto fit_result_blue = blue_spline_fitter.fit_open_spline();
    if (!fit_result_blue) {
        std::cerr << "Error fitting spline for blue cones: "
                  << SplineFitter::to_string(fit_result_blue.error()) << "\n";
        return 1;
    }

    auto length_result_blue = blue_spline_fitter.compute_spline_interval_lengths();
    if (!length_result_blue) {
        std::cerr << "Error computing spline interval lengths for blue cones: "
                  << SplineFitter::to_string(length_result_blue.error()) << "\n";
        return 1;
    }

    auto sample_result_blue = blue_spline_fitter.uniformly_sample_spline(resample_points);
    if (!sample_result_blue) {
        std::cerr << "Error sampling spline for blue cones: "
                  << SplineFitter::to_string(sample_result_blue.error()) << "\n";
        return 1;
    }

    SplineParametrization spline_interp_blue = sample_result_blue.value();

    // Yellow cones
    auto expected_yellow_spline_fitter = SplineFitter::create(yellow_cones, curv_weight);
    if (!expected_yellow_spline_fitter) {
        std::cerr << "Error creating SplineFitter for yellow cones: "
                  << SplineFitter::to_string(expected_yellow_spline_fitter.error()) << "\n";
        return 1;
    }

    SplineFitter yellow_spline_fitter = expected_yellow_spline_fitter.value();
    auto fit_result_yellow = yellow_spline_fitter.fit_open_spline();

    if (!fit_result_yellow) {
        std::cerr << "Error fitting spline for yellow cones: "
                  << SplineFitter::to_string(fit_result_yellow.error()) << "\n";
        return 1;
    }

    auto length_result_yellow = yellow_spline_fitter.compute_spline_interval_lengths();
    if (!length_result_yellow) {
        std::cerr << "Error computing spline interval lengths for yellow cones: "
                  << SplineFitter::to_string(length_result_yellow.error()) << "\n";
        return 1;
    }

    auto sample_result_yellow = yellow_spline_fitter.uniformly_sample_spline(resample_points);
    if (!sample_result_yellow) {
        std::cerr << "Error sampling spline for yellow cones: "
                  << SplineFitter::to_string(sample_result_yellow.error()) << "\n";
        return 1;
    }

    SplineParametrization spline_interp_yellow = sample_result_yellow.value();

    auto expected_center_line = compute_center_line(spline_interp_blue.X,
                                                    spline_interp_blue.Y,
                                                    spline_interp_yellow.X,
                                                    spline_interp_yellow.Y,
                                                    curv_weight,
                                                    resample_points,
                                                    verbose);
    if (!expected_center_line) {
        std::cerr << "Error computing center line: " << to_string(expected_center_line.error())
                  << "\n";
        return 1;
    }

    VectorPair center_line = expected_center_line.value();

    auto excpected_heading_blue =
        blue_spline_fitter.get_heading(spline_interp_blue.idx, spline_interp_blue.t);

    if (!excpected_heading_blue) {
        std::cerr << "Error computing heading for blue cones: "
                  << SplineFitter::to_string(excpected_heading_blue.error()) << "\n";
        return 1;
    }

    auto excpected_heading_yellow =
        yellow_spline_fitter.get_heading(spline_interp_yellow.idx, spline_interp_yellow.t);

    if (!excpected_heading_yellow) {
        std::cerr << "Error computing heading for yellow cones: "
                  << SplineFitter::to_string(excpected_heading_yellow.error()) << "\n";
        return 1;
    }

    auto excpected_curvature_blue =
        blue_spline_fitter.get_curvature(spline_interp_blue.idx, spline_interp_blue.t);

    if (!excpected_curvature_blue) {
        std::cerr << "Error computing curvature for blue cones: "
                  << SplineFitter::to_string(excpected_curvature_blue.error()) << "\n";
        return 1;
    }

    auto excpected_curvature_yellow =
        yellow_spline_fitter.get_curvature(spline_interp_yellow.idx, spline_interp_yellow.t);

    if (!excpected_curvature_yellow) {
        std::cerr << "Error computing curvature for yellow cones: "
                  << SplineFitter::to_string(excpected_curvature_yellow.error()) << "\n";
        return 1;
    }

    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Elapsed time: " << elapsed_seconds.count() << "s\n";

    // Output the interpolated points to CSV or visualize them
    // For example, write to CSV:
    std::ofstream file(path_output);
    file << "color,X,Y\n";
    for (int i = 0; i < spline_interp_blue.X.size(); ++i) {
        file << "blue," << spline_interp_blue.X(i) << "," << spline_interp_blue.Y(i) << "\n";
    }
    for (int i = 0; i < spline_interp_yellow.X.size(); ++i) {
        file << "yellow," << spline_interp_yellow.X(i) << "," << spline_interp_yellow.Y(i) << "\n";
    }
    for (int i = 0; i < center_line.first.size(); ++i) {
        file << "center," << center_line.first(i) << "," << center_line.second(i) << "\n";
    }
    file.close();

    return 0;
}
