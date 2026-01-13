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

#ifndef BRAINS2__TOOLS__GUI_STATE_HPP_
#define BRAINS2__TOOLS__GUI_STATE_HPP_

#include <string>
#include <vector>
#include <memory>
#include "brains2/common/track.hpp"

namespace brains2 {
namespace tools {

struct TrackInfo {
    std::string name;
    std::string filename;
    std::vector<double> s_vals;
    std::vector<double> X_vals;
    std::vector<double> Y_vals;
    std::vector<double> left_bound_X;
    std::vector<double> left_bound_Y;
    std::vector<double> right_bound_X;
    std::vector<double> right_bound_Y;
    double length;
    double min_s;
    double max_s;
};

class GuiState {
public:
    // Track management
    std::vector<TrackInfo> available_tracks;
    int selected_track_index;
    std::unique_ptr<common::Track> current_track;
    
    // Initial position
    double initial_s;
    double initial_n;
    double initial_psi;
    double initial_v;
    
    // Visualization settings
    bool show_track_bounds;
    bool show_centerline;
    bool show_start_position;
    int track_resolution;  // Number of points to sample for visualization
    
    GuiState();
    
    void load_available_tracks();
    void select_track(int index);
    void update_initial_s(double s);
    
private:
    void compute_track_bounds(TrackInfo& track_info);
};

}  // namespace tools
}  // namespace brains2

#endif  // BRAINS2__TOOLS__GUI_STATE_HPP_
