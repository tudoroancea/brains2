// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#ifndef TRACKS_HPP
#define TRACKS_HPP

#include <filesystem>
#include <string>
#include <unordered_map>
#include "brains2/common/cone_color.hpp"
#include "brains2/external/optional.hpp"
#include "Eigen/Dense"

namespace brains2 {
namespace common {

// ============================================================================
// functions to load/save cones and center line from the track_database package
// ============================================================================

/*
 * @brief Load cones from a CSV file with columns color, X, Y.
 *
 * @param track_path Path to the CSV file.
 * @return A map of cones with the color as key and the position as value.
 */
tl::optional<std::unordered_map<ConeColor, Eigen::MatrixX2d>> load_cones_from_file(
    const std::filesystem::path &track_path);

/*
 * @brief Load cones from the track database with the name track_name.
 *
 * @param track_name Name of the track.
 * @return A map of cones with the color as key and the position as value.
 */
tl::optional<std::unordered_map<ConeColor, Eigen::MatrixX2d>> load_cones_from_track_database(
    const std::string &track_name);

/*
 * @brief Save cones to a CSV file with columns color, X, Y.
 *
 * @param track_path Path to the CSV file.
 * @param cones_map Map of cones with the color as key and the position as value.
 */
void save_cones(const std::filesystem::path &track_path,
                const std::unordered_map<ConeColor, Eigen::MatrixX2d> &cones_map);

// void load_center_line(const std::string &track_name_or_file,
//                       Eigen::MatrixX2d &center_line,
//                       Eigen::MatrixX2d &track_widths);
//
// void save_center_line(const std::string &filename,
//                       const Eigen::MatrixX2d &center_line,
//                       const Eigen::MatrixX2d &track_widths);

// =================================================================
// class used to wrap the track files generated in python
// =================================================================

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
