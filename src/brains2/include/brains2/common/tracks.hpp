// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#ifndef TRACKS_HPP
#define TRACKS_HPP

#include <filesystem>
#include <string>
#include <tuple>
#include <unordered_map>
#include "brains2/common/cone_color.hpp"
#include "brains2/external/optional.hpp"
#include "Eigen/Dense"

namespace brains2 {
namespace common {

// =================================================================
// class used to wrap the track files generated in python
// =================================================================
// TODO: leave this class in track.hpp
/*
 * @brief Class representing a portion of the track
 */
class Track {
private:
    // TODO(mattbrth): merge right_width and left_width into one vector track_width
    Eigen::VectorXd s_ref, X_ref, Y_ref, phi_ref, kappa_ref, right_width, left_width;
    Eigen::VectorXd delta_s;
    // TODO(mattbrth): same for coeffs
    Eigen::MatrixX2d coeffs_X, coeffs_Y, coeffs_phi, coeffs_kappa, coeffs_right_width,
        coeffs_left_width;

    void interp(const Eigen::MatrixXd &coeffs, double s, double &value, int ind = -1) const;

public:
    explicit Track(const std::string &csv_file);

    explicit Track(const std::vector<double> &s_ref,
                   const std::vector<double> &X_ref,
                   const std::vector<double> &Y_ref,
                   const std::vector<double> &phi_ref,
                   const std::vector<double> &kappa_ref,
                   const std::vector<double> &right_width,
                   const std::vector<double> &left_width);


    // TODO: think of a better way to optionally return certain elements
    void project(const Eigen::Vector2d &car_pos,
                 double s_guess,
                 double s_tol,
                 double *s = nullptr,
                 double *X_ref_proj = nullptr,
                 double *Y_ref_proj = nullptr,
                 double *phi_ref_proj = nullptr,
                 double *phi_ref_preview = nullptr,
                 double *kappa_ref_proj = nullptr,
                 double *right_width_proj = nullptr,
                 double *left_width_proj = nullptr);

    void frenet_to_cartesian(const double &s, const double &n, double &X, double &Y) const;
    void frenet_to_cartesian(const Eigen::VectorXd &s,
                             const Eigen::VectorXd &n,
                             Eigen::VectorXd &X,
                             Eigen::VectorXd &Y) const;

    double length() const;
    size_t size() const;

    double *get_s_ref();
    double *get_kappa_ref();
    double *get_X_ref();
    double *get_Y_ref();
};

}  // namespace common
}  // namespace brains2

#endif  // TRACKS_HPP
