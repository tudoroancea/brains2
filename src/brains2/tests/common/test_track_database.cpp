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

// Possible tests:
// 1. load all the cones in the track database
// 2. create a tmp file and save the cones in it

#include <filesystem>
#include "brains2/common/track_database.hpp"
#include "gtest/gtest.h"

TEST(TrackDatabaseTestSuite, load_cones_track_database_test) {
    for (const std::string track_name : {"alpha", "beta", "gamma"}) {
        tl::optional<std::unordered_map<brains2::common::ConeColor, Eigen::MatrixX2d>> cones =
            brains2::common::load_cones_from_track_database(track_name);
        EXPECT_TRUE(cones.has_value());
    }
}

TEST(TrackDatabaseTestSuite, load_cones_file_test) {
    for (const std::string track_name : {"alpha", "beta", "gamma"}) {
        std::filesystem::path track_path = TRACK_DATABASE_PATH;
        track_path /= (track_name + ".csv");
        tl::optional<std::unordered_map<brains2::common::ConeColor, Eigen::MatrixX2d>> cones =
            brains2::common::load_cones_from_file(track_path);
        EXPECT_TRUE(cones.has_value());
    }
}

TEST(TrackDatabaseTestSuite, save_cones_test) {
    // create tmp folder
    std::filesystem::path tmp_dir = std::filesystem::temp_directory_path();
    std::filesystem::create_directory(tmp_dir / "brains2_test");
    std::filesystem::path tmp_file = tmp_dir / "brains2_test" / "test_cones.csv";

    // save cones
    std::unordered_map<brains2::common::ConeColor, Eigen::MatrixX2d> cones_map;
    cones_map[brains2::common::ConeColor::YELLOW] = Eigen::MatrixX2d::Zero(2, 2);
    cones_map[brains2::common::ConeColor::BLUE] = Eigen::MatrixX2d::Zero(2, 2);
    cones_map[brains2::common::ConeColor::ORANGE] = Eigen::MatrixX2d::Zero(2, 2);
    brains2::common::save_cones(tmp_file, cones_map);

    // load cones and check if they are the same
    tl::optional<std::unordered_map<brains2::common::ConeColor, Eigen::MatrixX2d>> cones =
        brains2::common::load_cones_from_file(tmp_file);
    ASSERT_TRUE(cones.has_value());
    EXPECT_EQ(cones->size(), 3);
    EXPECT_EQ(cones->at(brains2::common::ConeColor::YELLOW), Eigen::MatrixX2d::Zero(2, 2));
    EXPECT_EQ(cones->at(brains2::common::ConeColor::BLUE), Eigen::MatrixX2d::Zero(2, 2));
    EXPECT_EQ(cones->at(brains2::common::ConeColor::ORANGE), Eigen::MatrixX2d::Zero(2, 2));
}
