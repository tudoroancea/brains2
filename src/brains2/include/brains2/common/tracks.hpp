// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#ifndef TRACKS_HPP
#define TRACKS_HPP

#include <filesystem>
#include <tuple>
#include "brains2/external/optional.hpp"
#include "Eigen/Dense"

namespace brains2 {
namespace common {

/*
 * @brief Class representing a portion of the track
 */
class Track {
private:
    // Values taken by the splines
    Eigen::VectorXd vals_s, vals_X, vals_Y, vals_phi, vals_kappa, vals_width;
    // Spline coefficients cached to avoid recomputation
    Eigen::MatrixX2d coeffs_X, coeffs_Y, coeffs_phi, coeffs_kappa, coeffs_width;
    /*
     * @brief Evaluates the spline based on the provided coefficients (against the track progress s)
     */
    double interp(const Eigen::MatrixXd &coeffs, double s, int ind = -1) const;

    Track() = default;

public:
    static tl::optional<Track> from_values(const std::vector<double> &s,
                                           const std::vector<double> &X,
                                           const std::vector<double> &Y,
                                           const std::vector<double> &phi,
                                           const std::vector<double> &kappa,
                                           const std::vector<double> &width);
    static tl::optional<Track> from_file(const std::filesystem::path &csv_file_path);

    /*
     *  @brief Returns the number of points in the track
     */
    size_t size() const;
    /*
     *  @brief Returns the length of the track in meters
     */
    double length() const;

    /*
     *  @brief Returns the minimum progress s on the track
     */
    double s_min() const;

    /*
     *  @brief Evaluates the track's X coordinate at a progress s
     */
    double eval_X(double s) const;
    /*
     *  @brief Evaluates the track's Y coordinate at a progress s
     */
    double eval_Y(double s) const;
    /*
     *  @brief Evaluates the track's phi angle at a progress s
     */
    double eval_phi(double s) const;
    /*
     *  @brief Evaluates the track's curvature at a progress s
     */
    double eval_kappa(double s) const;
    /*
     *  @brief Evaluates the track's width at a progress s
     */
    double eval_width(double s) const;

    /*
     * @brief Get a constant reference to the track's progress values
     */
    const Eigen::VectorXd &get_vals_s() const;
    /*
     * @brief Get a constant reference to the track's X coordinate values
     */
    const Eigen::VectorXd &get_vals_X() const;
    /*
     * @brief Get a constant reference to the track's Y coordinate values
     */
    const Eigen::VectorXd &get_vals_Y() const;
    /*
     * @brief Get a constant reference to the track's phi angle values
     */
    const Eigen::VectorXd &get_vals_phi() const;
    /*
     * @brief Get a constant reference to the track's curvature values
     */
    const Eigen::VectorXd &get_vals_kappa() const;
    /*
     * @brief Get a constant reference to the track's width values
     */
    const Eigen::VectorXd &get_vals_width() const;

    size_t find_interval(double s) const;

    /*
     *  @brief Converts a Frenet coordinate to a Cartesian coordinate
     */
    Eigen::Vector2d frenet_to_cartesian(const double &s, const double &n) const;

    /*
         * @brief Compute orthogonal projection of a point onto the track
         * @param X The X coordinate of the point
         * @param Y The Y coordinate of the point
         * @param s_guess An initial guess for the progress s
         * @param s_tol A tolerance such that the point will be searched only
                        within the range [s_guess - s_tol, s_guess + s_tol]
         * @return A tuple containing the progress s and the projected position
         */
    std::tuple<double, Eigen::Vector2d> project(double X,
                                                double Y,
                                                double s_guess,
                                                double s_tol) const;
};

}  // namespace common
}  // namespace brains2

#endif  // TRACKS_HPP
