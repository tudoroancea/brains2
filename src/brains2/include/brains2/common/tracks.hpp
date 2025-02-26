// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#ifndef TRACKS_HPP
#define TRACKS_HPP

#include <filesystem>
#include <tuple>
#include "brains2/external/expected.hpp"
#include "brains2/external/optional.hpp"
#include "Eigen/Dense"

namespace brains2 {
namespace common {

struct CartesianPose {
    double X, Y, phi;
};

struct FrenetPose {
    double s, n, psi;
};

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
    /*
     * @brief Possible errors when constructing a Track object.
     */
    enum class Error {
        DIFFERENT_SIZES,        // Provided arrays of values have different sizes
        NONMONOTONIC_PROGRESS,  // Provided progress values are not strictly increasing
        NEGATIVE_WIDTH,         // Provided width values are sometimes negative
        DISCONTINUOUS_HEADING,  // Provided heading values are not continuous
        FILE_NOT_FOUND,         // CSV file not found
    };

    /*
     * @brief Constructs a Track object from the provided values.
     * @param s The progress values.
     * @param X The x-coordinates.
     * @param Y The y-coordinates.
     * @param phi The heading angles.
     * @param kappa The curvature values.
     * @param width The width values (here half the track width).
     * @return A tl::expected object containing the Track object or an Error.
     */
    static tl::expected<Track, Error> from_values(const std::vector<double> &s,
                                                  const std::vector<double> &X,
                                                  const std::vector<double> &Y,
                                                  const std::vector<double> &phi,
                                                  const std::vector<double> &kappa,
                                                  const std::vector<double> &width);
    /*
     * @brief Constructs a Track object from the provided values.
     * @param s The progress values.
     * @param X The x-coordinates.
     * @param Y The y-coordinates.
     * @param phi The heading angles.
     * @param kappa The curvature values.
     * @param width The width values (here half the track width).
     * @return A tl::expected object containing the Track object or an Error.
     */
    static tl::expected<Track, Error> from_values(const Eigen::VectorXd &s,
                                                  const Eigen::VectorXd &X,
                                                  const Eigen::VectorXd &Y,
                                                  const Eigen::VectorXd &phi,
                                                  const Eigen::VectorXd &kappa,
                                                  const Eigen::VectorXd &width);
    /*
     * @brief Constructs a Track object from a CSV file.
     * @param csv_file_path The path to the CSV file containing 6 columns: s, X, Y, phi, kappa,
     * width
     * @return A tl::expected object containing the Track object or an Error.
     */
    static tl::expected<Track, Error> from_file(const std::filesystem::path &csv_file_path);

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

    /*
     * @brief Find the index of the spline interval containing the given track progress
     */
    size_t find_interval(double s) const;

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

    /*
     *  @brief Converts a pose in Frenet coordinates to Cartesian coordinates and returns the
     *         corresponding projection on the track.
     */
    std::pair<CartesianPose, CartesianPose> frenet_to_cartesian(
        const FrenetPose &frenet_pose) const;

    /*
     *  @brief Converts a pose in Cartesian coordinates to Frenet coordinates and returns the
     *         corresponding projection on the track.
     */
    std::pair<FrenetPose, CartesianPose> cartesian_to_frenet(
        const CartesianPose &cartesian_pose,
        tl::optional<double> s_guess = tl::nullopt,
        tl::optional<double> s_tol = tl::nullopt) const;
};

std::string to_string(const Track::Error &error);

}  // namespace common
}  // namespace brains2

#endif  // TRACKS_HPP
