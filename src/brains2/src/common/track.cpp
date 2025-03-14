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

#include "brains2/common/track.hpp"
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>
#include "brains2/common/math.hpp"
#include "brains2/external/icecream.hpp"
#include "brains2/external/optional.hpp"
#include "brains2/external/rapidcsv.hpp"
#include "Eigen/src/Core/Matrix.h"

namespace brains2::common {

tl::expected<Track, Track::Error> Track::from_values(const std::vector<double>& s,
                                                     const std::vector<double>& X,
                                                     const std::vector<double>& Y,
                                                     const std::vector<double>& phi,
                                                     const std::vector<double>& kappa,
                                                     const std::vector<double>& width) {
    return Track::from_values(Eigen::VectorXd::Map(s.data(), s.size()),
                              Eigen::VectorXd::Map(X.data(), X.size()),
                              Eigen::VectorXd::Map(Y.data(), Y.size()),
                              Eigen::VectorXd::Map(phi.data(), phi.size()),
                              Eigen::VectorXd::Map(kappa.data(), kappa.size()),
                              Eigen::VectorXd::Map(width.data(), width.size()));
}

tl::expected<Track, Track::Error> Track::from_values(const Eigen::VectorXd& s,
                                                     const Eigen::VectorXd& X,
                                                     const Eigen::VectorXd& Y,
                                                     const Eigen::VectorXd& phi,
                                                     const Eigen::VectorXd& kappa,
                                                     const Eigen::VectorXd& width) {
    // Check all inputs have the same size
    const auto size = s.size();
    if (size != X.size() || size != Y.size() || size != phi.size() || size != kappa.size() ||
        size != width.size()) {
        return tl::unexpected(Track::Error::DIFFERENT_SIZES);
    }
    // Check that s is increasing s_{i+1} <= s_i
    if ((s.tail(size - 1).array() <= s.head(size - 1).array()).any()) {
        return tl::unexpected(Track::Error::NONMONOTONIC_PROGRESS);
    }
    // Check that the width is positive
    if ((width.array() <= 0).any()) {
        return tl::unexpected(Track::Error::NONPOSITIVE_WIDTH);
    }
    // Check that the heading is continuous
    std::vector<double> diffs(size);
    std::adjacent_difference(phi.begin(), phi.end(), diffs.begin());
    // NOTE: here diffs[0] == phi[0], diffs[1] == phi[1] - phi[0], etc.
    if (std::any_of(next(diffs.begin()), diffs.end(), [](auto x) { return x > M_PI; })) {
        return tl::unexpected(Track::Error::DISCONTINUOUS_HEADING);
    }

    // Copy the data
    Track track{};
    track.vals_s = s;
    track.vals_X = X;
    track.vals_Y = Y;
    track.vals_phi = phi;
    track.vals_kappa = kappa;
    track.vals_width = width;

    // Compute the differences in progress s
    auto delta_s =
        track.vals_s.tail(track.vals_s.size() - 1) - track.vals_s.head(track.vals_s.size() - 1);

    // fit linear splines
    track.coeffs_X.resize(size - 1, 2);
    track.coeffs_Y.resize(size - 1, 2);
    track.coeffs_phi.resize(size - 1, 2);
    track.coeffs_kappa.resize(size - 1, 2);
    track.coeffs_width.resize(size - 1, 2);
    for (Eigen::Index i = 0; i < size - 1; ++i) {
        track.coeffs_X(i, 0) = track.vals_X(i);
        track.coeffs_X(i, 1) = (track.vals_X(i + 1) - track.vals_X(i)) / delta_s(i);

        track.coeffs_Y(i, 0) = track.vals_Y(i);
        track.coeffs_Y(i, 1) = (track.vals_Y(i + 1) - track.vals_Y(i)) / delta_s(i);

        track.coeffs_phi(i, 0) = track.vals_phi(i);
        track.coeffs_phi(i, 1) = (track.vals_phi(i + 1) - track.vals_phi(i)) / delta_s(i);

        track.coeffs_kappa(i, 0) = track.vals_kappa(i);
        track.coeffs_kappa(i, 1) = (track.vals_kappa(i + 1) - track.vals_kappa(i)) / delta_s(i);

        track.coeffs_width(i, 0) = track.vals_width(i);
        track.coeffs_width(i, 1) = (track.vals_width(i + 1) - track.vals_width(i)) / delta_s(i);
    }
    return track;
}

tl::expected<Track, Track::Error> Track::from_file(const std::filesystem::path& csv_file) {
    if (!std::filesystem::exists(csv_file)) {
        return tl::make_unexpected(Track::Error::FILE_NOT_FOUND);
    }
    rapidcsv::Document doc(csv_file.string());
    return Track::from_values(doc.GetColumn<double>("s"),
                              doc.GetColumn<double>("X"),
                              doc.GetColumn<double>("Y"),
                              doc.GetColumn<double>("phi"),
                              doc.GetColumn<double>("kappa"),
                              doc.GetColumn<double>("w"));
}

size_t Track::size() const {
    return vals_s.size();
}

double Track::length() const {
    // This class usually represents 'open' tracks. When it is used to represent a closed track, the
    // length will be slightly off as we should add
    //  std::hypot(vals_X(size - 1) - vals_X(0), vals_Y(size - 1) - vals_Y(0))
    return vals_s(vals_s.size() - 1) - vals_s(0);
}

double Track::s_min() const {
    return vals_s(0);
}

static double angle3pt(const Eigen::Vector2d& a,
                       const Eigen::Vector2d& b,
                       const Eigen::Vector2d& c) {
    return wrap_to_pi(std::atan2(c(1) - b(1), c(0) - b(0)) - std::atan2(a(1) - b(1), a(0) - b(0)));
}

double Track::interp(const Eigen::MatrixXd& coeffs, double s, int ind) const {
    // find i such that s_ref[i] <= s < s_ref[i+1]
    if (ind < 0) {
        ind = find_interval(s);
    }
    // find the value of the spline at s
    return coeffs(ind, 0) + coeffs(ind, 1) * (s - vals_s(ind));
}

size_t Track::find_interval(double s) const {
    return std::upper_bound(vals_s.data(), vals_s.data() + vals_s.size(), s) - vals_s.data() - 1;
}

std::tuple<double, Eigen::Vector2d> Track::project(double X,
                                                   double Y,
                                                   double s_guess,
                                                   double s_tol) const {
    // extract all the points in X_ref, Y_ref associated with s_ref values
    // within s_guess +- s_tol
    double s_low = std::max(s_guess - s_tol, vals_s(0)),
           s_up = std::min(s_guess + s_tol, vals_s(vals_s.size() - 1));
    int64_t id_low = find_interval(s_low), id_up = find_interval(s_up);
    if (id_low > 0) {
        --id_low;
    }
    if (id_up < static_cast<int64_t>(vals_s.size()) - 1) {
        ++id_up;
    }
    Eigen::ArrayX2d local_traj =
        Eigen::ArrayX2d::Zero(id_up - id_low + 1,
                              2);  // problem with difference of size_ints ?
    local_traj.col(0) = vals_X.segment(id_low, id_up - id_low + 1);
    local_traj.col(1) = vals_Y.segment(id_low, id_up - id_low + 1);

    // find the closest point to car_pos to find one segment extremity
    Eigen::VectorXd sqdist = (local_traj.col(0) - X).square() + (local_traj.col(1) - Y).square();
    int64_t id_min, id_prev, id_next;
    sqdist.minCoeff(&id_min);
    id_prev = id_min - 1;
    if (id_min == 0) {
        id_prev = local_traj.rows() - 1;
    }
    id_next = id_min + 1;
    if (id_min == static_cast<int64_t>(local_traj.rows()) - 1) {
        id_next = 0;
    }
    // TODO: what happens if id_min == 0 or id_min == local_traj.rows() - 1 ?
    // This should not happen though

    // compute the angles between car_pos, the closest point and the next and
    // previous point to find the second segment extremity
    double angle_prev = std::fabs(
               angle3pt(local_traj.row(id_min), Eigen::Vector2d(X, Y), local_traj.row(id_prev))),
           angle_next = std::fabs(
               angle3pt(local_traj.row(id_min), Eigen::Vector2d(X, Y), local_traj.row(id_next)));
    Eigen::Vector2d a, b;
    double sa, sb;
    if (angle_prev > angle_next) {
        a = local_traj.row(id_prev);
        b = local_traj.row(id_min);
        sa = vals_s(id_prev + id_low);
        sb = vals_s(id_min + id_low);
    } else {
        a = local_traj.row(id_min);
        b = local_traj.row(id_next);
        sa = vals_s(id_min + id_low);
        sb = vals_s(id_next + id_low);
    }

    // project car_pos on the segment and retrieve lambda
    double dx = b(0) - a(0), dy = b(1) - a(1);
    double lambda = ((X - a(0)) * dx + (Y - a(1)) * dy) / (dx * dx + dy * dy);

    // compute the interpolated values (with non null pointers) at lambda using
    // the index of the closest point
    double s_proj = sa + lambda * (sb - sa);
    double X_ref_proj = a(0) + lambda * (b(0) - a(0));
    double Y_ref_proj = a(1) + lambda * (b(1) - a(1));
    return std::make_tuple(s_proj, Eigen::Vector2d(X_ref_proj, Y_ref_proj));
}

double Track::eval_X(double s) const {
    return this->interp(this->coeffs_X, s);
}

double Track::eval_Y(double s) const {
    return this->interp(this->coeffs_Y, s);
}

double Track::eval_phi(double s) const {
    return this->interp(this->coeffs_phi, s);
}

double Track::eval_kappa(double s) const {
    return this->interp(this->coeffs_kappa, s);
}

double Track::eval_width(double s) const {
    return this->interp(this->coeffs_width, s);
}

const Eigen::VectorXd& Track::get_vals_s() const {
    return this->vals_s;
}

const Eigen::VectorXd& Track::get_vals_X() const {
    return this->vals_X;
}

const Eigen::VectorXd& Track::get_vals_Y() const {
    return this->vals_Y;
}

const Eigen::VectorXd& Track::get_vals_phi() const {
    return this->vals_phi;
}

const Eigen::VectorXd& Track::get_vals_kappa() const {
    return this->vals_kappa;
}

const Eigen::VectorXd& Track::get_vals_width() const {
    return this->vals_width;
}

std::pair<CartesianPose, CartesianPose> Track::frenet_to_cartesian(
    const FrenetPose& frenet_pose) const {
    const double X_proj = this->eval_X(frenet_pose.s);
    const double Y_proj = this->eval_Y(frenet_pose.s);
    const double phi_proj = this->eval_phi(frenet_pose.s);
    const double X = X_proj - frenet_pose.n * sin(phi_proj);
    const double Y = Y_proj + frenet_pose.n * cos(phi_proj);
    const double psi = frenet_pose.psi + phi_proj;
    return std::make_pair(CartesianPose{X, Y, psi}, CartesianPose{X, Y, psi});
}

std::pair<FrenetPose, CartesianPose> Track::cartesian_to_frenet(const CartesianPose& cartesian_pose,
                                                                tl::optional<double> s_guess,
                                                                tl::optional<double> s_tol) const {
    const auto [s, pos_proj] = this->project(cartesian_pose.X,
                                             cartesian_pose.Y,
                                             s_guess ? *s_guess : this->s_min(),
                                             s_tol ? *s_tol : this->length());
    const auto phi_proj = this->eval_phi(s);
    const double n = -(cartesian_pose.X - pos_proj(0)) * sin(phi_proj) +
                     (cartesian_pose.Y - pos_proj(1)) * cos(phi_proj);
    const double psi = wrap_to_pi(cartesian_pose.phi - phi_proj);
    return std::make_pair(FrenetPose{s, n, psi}, CartesianPose{pos_proj(0), pos_proj(1), phi_proj});
}

std::string to_string(const Track::Error& error) {
    switch (error) {
        case Track::Error::DIFFERENT_SIZES:
            return "Different sizes";
        case Track::Error::NONMONOTONIC_PROGRESS:
            return "Nonmonotonic track progress";
        case Track::Error::NONPOSITIVE_WIDTH:
            return "Nonpositive track width";
        case Track::Error::DISCONTINUOUS_HEADING:
            return "Discontinuous heading";
        case Track::Error::FILE_NOT_FOUND:
            return "CSV File not found";
    }
}

}  // namespace brains2::common
