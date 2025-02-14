// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include <filesystem>
#include <vector>
#include "brains2/common/track_database.hpp"
#include "brains2/external/rapidcsv.hpp"

namespace brains2::common {

static tl::optional<std::string> validate_track_name(const std::string& track_name) {
#ifdef TRACK_DATABASE_PATH
    std::filesystem::path track_file(TRACK_DATABASE_PATH);
    track_file /= (track_name + ".csv");
    if (std::filesystem::exists(track_file)) {
        return track_file.string();
    } else {
        std::cerr << "Track " << track_name << " not found in TRACK_DATABASE_PATH" << std::endl;
        return tl::nullopt;
    }
#else
#error TRACK_DATABASE_PATH not defined
#endif
}

tl::optional<std::unordered_map<ConeColor, Eigen::MatrixX2d>> load_cones_from_file(
    const std::filesystem::path& track_path) {
    rapidcsv::Document cones(track_path.string());
    // get the positions of the cones in columns X and Y and the corresponding
    // type in column cone_type
    std::vector<double> cones_X = cones.GetColumn<double>("X");
    std::vector<double> cones_Y = cones.GetColumn<double>("Y");
    std::vector<std::string> cones_type = cones.GetColumn<std::string>("color");
    std::unordered_map<ConeColor, Eigen::MatrixX2d> cones_map;
    for (size_t i = 0; i < cones_X.size(); ++i) {
        ConeColor type = cone_color_from_string(cones_type[i]);
        if (cones_map.find(type) == cones_map.end()) {
            cones_map[type] = Eigen::MatrixX2d::Zero(0, 2);
        }
        cones_map[type].conservativeResize(cones_map[type].rows() + 1, 2);
        cones_map[type].bottomRows<1>() << cones_X[i], cones_Y[i];
    }
    return cones_map;
}

tl::optional<std::unordered_map<ConeColor, Eigen::MatrixX2d>> load_cones_from_track_database(
    const std::string& track_name) {
    tl::optional<std::string> track_name_opt = validate_track_name(track_name);
    if (!track_name_opt) {
        return tl::nullopt;
    }
    return load_cones_from_file(track_name_opt.value());
}

void save_cones(const std::filesystem::path& track_path,
                const std::unordered_map<ConeColor, Eigen::MatrixX2d>& cones_map) {
    std::ofstream f(track_path.string());
    f << "color,X,Y\n";

    for (auto it = cones_map.begin(); it != cones_map.end(); ++it) {
        for (Eigen::Index id(0); id < it->second.rows(); ++id) {
            f << cone_color_to_string(it->first) << "," << it->second(id, 0) << ","
              << it->second(id, 1) << "\n";
        }
    }
    f.close();
}

}  // namespace brains2::common
