// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#ifndef TRACK_DATABASE_HPP
#define TRACK_DATABASE_HPP

#include <filesystem>
#include <string>
#include <unordered_map>
#include "brains2/common/cone_color.hpp"
#include "brains2/external/optional.hpp"
#include "Eigen/Dense"

namespace brains2 {
namespace common {

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

}  // namespace common
}  // namespace brains2

#endif  // TRACK_DATABASE_HPP
