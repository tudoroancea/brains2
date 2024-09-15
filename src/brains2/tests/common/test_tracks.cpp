// Possible tests:
// 1. load all the cones in the track database
// 2. create a tmp file and save the cones in it

#include <filesystem>
#include "brains2/common/tracks.hpp"
#include "gtest/gtest.h"

TEST(TracksTestSuite, bruh) {
    std::cout << "hello world" << std::endl;
}

TEST(TracksTestSuite, bruh2) {
    FAIL() << "bruh";
}

// TEST(TracksTestSuite, load_cones_test) {
//     for (const std::string &track_name : {"alpha", "beta", "gamma"}) {
//         tl::optional<std::unordered_map<brains2::common::ConeColor, Eigen::MatrixX2d>> cones =
//             brains2::common::load_cones(track_name);
//         EXPECT_TRUE(cones.has_value());
//     }
// }
//
// TEST(TracksTestSuite, save_cones_test) {
//     // create tmp folder
//     std::filesystem::path tmp_dir = std::filesystem::temp_directory_path();
//     std::filesystem::create_directory(tmp_dir / "brains2_test");
//     std::filesystem::path tmp_file = tmp_dir / "brains2_test" / "test_cones.csv";
//
//     // save cones
//     std::unordered_map<brains2::common::ConeColor, Eigen::MatrixX2d> cones_map;
//     cones_map[brains2::common::ConeColor::YELLOW] = Eigen::MatrixX2d::Zero(2, 2);
//     cones_map[brains2::common::ConeColor::BLUE] = Eigen::MatrixX2d::Zero(2, 2);
//     cones_map[brains2::common::ConeColor::ORANGE] = Eigen::MatrixX2d::Zero(2, 2);
//     brains2::common::save_cones(tmp_file.string(), cones_map);
//
//     // load cones and check if they are the same
//     tl::optional<std::unordered_map<brains2::common::ConeColor, Eigen::MatrixX2d>> cones =
//         brains2::common::load_cones(tmp_file.string());
//     ASSERT_TRUE(cones.has_value());
//     EXPECT_EQ(cones->size(), 3);
//     EXPECT_EQ(cones->at(brains2::common::ConeColor::YELLOW), Eigen::MatrixX2d::Zero(2, 2));
//     EXPECT_EQ(cones->at(brains2::common::ConeColor::BLUE), Eigen::MatrixX2d::Zero(2, 2));
//     EXPECT_EQ(cones->at(brains2::common::ConeColor::ORANGE), Eigen::MatrixX2d::Zero(2, 2));
// }
