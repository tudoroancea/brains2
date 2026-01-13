// Copyright 2025 Tudor Oancea, Mateo Berthet
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "gui_state.hpp"
#include <filesystem>
#include <iostream>
#include <cmath>

namespace fs = std::filesystem;

namespace brains2 {
namespace tools {

GuiState::GuiState()
    : selected_track_index(0),
      initial_s(0.0),
      initial_n(0.0),
      initial_psi(0.0),
      initial_v(3.0),
      show_track_bounds(true),
      show_centerline(true),
      show_start_position(true),
      track_resolution(500) {
    load_available_tracks();
    if (!available_tracks.empty()) {
        select_track(0);
    }
}

void GuiState::load_available_tracks() {
    available_tracks.clear();
    
    // Track names and their CSV file names
    std::vector<std::pair<std::string, std::string>> track_defs = {
        {"alpha", "alpha_center_line.csv"},
        {"beta", "beta_center_line.csv"},
        {"gamma", "gamma_center_line.csv"}
    };
    
    for (const auto& [name, filename] : track_defs) {
        TrackInfo track_info;
        track_info.name = name;
        track_info.filename = filename;
        
        // Try to load the track from track_database
        fs::path track_db_path(TRACK_DATABASE_PATH);
        fs::path csv_path = track_db_path / filename;
        
        if (fs::exists(csv_path)) {
            auto result = common::Track::from_file(csv_path);
            if (result.has_value()) {
                current_track = std::make_unique<common::Track>(std::move(result.value()));
                
                // Store track values for plotting
                const auto& s_vals = current_track->get_vals_s();
                const auto& X_vals = current_track->get_vals_X();
                const auto& Y_vals = current_track->get_vals_Y();
                
                track_info.s_vals.resize(static_cast<size_t>(s_vals.size()));
                track_info.X_vals.resize(static_cast<size_t>(X_vals.size()));
                track_info.Y_vals.resize(static_cast<size_t>(Y_vals.size()));
                
                for (size_t i = 0; i < static_cast<size_t>(s_vals.size()); i++) {
                    track_info.s_vals[i] = s_vals(static_cast<Eigen::Index>(i));
                    track_info.X_vals[i] = X_vals(static_cast<Eigen::Index>(i));
                    track_info.Y_vals[i] = Y_vals(static_cast<Eigen::Index>(i));
                }
                
                track_info.length = current_track->length();
                track_info.min_s = current_track->s_min();
                track_info.max_s = track_info.min_s + track_info.length;

                // Compute track bounds using the track object
                compute_track_bounds(track_info, *current_track);

                available_tracks.push_back(track_info);
            } else {
                std::cerr << "Failed to load track " << name << ": " << common::to_string(result.error()) << std::endl;
            }
        } else {
            std::cerr << "Track file not found: " << csv_path << std::endl;
        }
    }
}

void GuiState::compute_track_bounds(TrackInfo& track_info, const common::Track& track) {
    const size_t n_points = track_info.s_vals.size();

    track_info.left_bound_X.resize(n_points);
    track_info.left_bound_Y.resize(n_points);
    track_info.right_bound_X.resize(n_points);
    track_info.right_bound_Y.resize(n_points);

    // Get phi and width values directly from the track object
    const auto& phi_vals = track.get_vals_phi();
    const auto& width_vals = track.get_vals_width();

    // Verify sizes match
    if (static_cast<size_t>(phi_vals.size()) != n_points || static_cast<size_t>(width_vals.size()) != n_points) {
        std::cerr << "Size mismatch: n_points=" << n_points
                  << ", phi_vals.size()=" << phi_vals.size()
                  << ", width_vals.size()=" << width_vals.size() << std::endl;
        return;
    }

    for (size_t i = 0; i < n_points; i++) {
        double X = track_info.X_vals[i];
        double Y = track_info.Y_vals[i];
        double phi = phi_vals(static_cast<Eigen::Index>(i));
        double width = width_vals(static_cast<Eigen::Index>(i));

        // Compute left and right bounds (perpendicular to centerline)
        double cos_phi = std::cos(phi + M_PI / 2.0);
        double sin_phi = std::sin(phi + M_PI / 2.0);

        // Left bound (positive lateral offset)
        track_info.left_bound_X[i] = X + width * cos_phi;
        track_info.left_bound_Y[i] = Y + width * sin_phi;

        // Right bound (negative lateral offset)
        track_info.right_bound_X[i] = X - width * cos_phi;
        track_info.right_bound_Y[i] = Y - width * sin_phi;
    }
}

void GuiState::select_track(int index) {
    if (index < 0 || index >= static_cast<int>(available_tracks.size())) {
        return;
    }
    
    selected_track_index = index;
    
    // Reload the track
    const TrackInfo& track_info = available_tracks[index];
    fs::path track_db_path(TRACK_DATABASE_PATH);
    fs::path csv_path = track_db_path / track_info.filename;
    
    auto result = common::Track::from_file(csv_path);
    if (result.has_value()) {
        current_track = std::make_unique<common::Track>(std::move(result.value()));
        // Reset initial position to start of track
        initial_s = current_track->s_min() + 5.0;  // Start a bit into the track
        initial_n = 0.0;
        initial_psi = 0.0;
        std::cout << "Selected track: " << track_info.name << std::endl;
    } else {
        std::cerr << "Failed to load track: " << common::to_string(result.error()) << std::endl;
    }
}

void GuiState::update_initial_s(double s) {
    if (current_track) {
        // Clamp to track bounds
        double min_s = current_track->s_min();
        double max_s = min_s + current_track->length() - 1.0;  // Leave some margin
        initial_s = std::max(min_s, std::min(s, max_s));
    } else {
        initial_s = s;
    }
}

}  // namespace tools
}  // namespace brains2
