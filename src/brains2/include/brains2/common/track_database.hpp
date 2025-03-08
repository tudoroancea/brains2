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
