// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include "brains2/common/tracks.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <stdexcept>
#include <vector>
#include "brains2/common/math.hpp"
#include "brains2/external/icecream.hpp"
#include "brains2/external/rapidcsv.hpp"

namespace brains2::common {

tl::optional<Track> Track::from_values(const std::vector<double>& s,
                                       const std::vector<double>& X,
                                       const std::vector<double>& Y,
                                       const std::vector<double>& phi,
                                       const std::vector<double>& kappa,
                                       const std::vector<double>& width) {
    // Check that all the vectors have the same size
    const size_t size = s.size();
    if (size != X.size() || size != Y.size() || size != phi.size() || size != kappa.size() ||
        size != width.size()) {
        return tl::nullopt;
    }
    // Copy the data in the Eigen vectors
    // (the Eigen::Map object creates a view to the data in the std::vector,
    //  but assigning it to an Eigen::Vector copies it)
    Track track{};
    track.vals_s = Eigen::VectorXd::Map(s.data(), s.size());
    track.vals_X = Eigen::VectorXd::Map(X.data(), X.size());
    track.vals_Y = Eigen::VectorXd::Map(Y.data(), Y.size());
    track.vals_phi = Eigen::VectorXd::Map(phi.data(), phi.size());
    track.vals_kappa = Eigen::VectorXd::Map(kappa.data(), kappa.size());
    track.vals_width = Eigen::VectorXd::Map(width.data(), width.size());

    // Compute the differences in progress s
    auto delta_s =
        track.vals_s.tail(track.vals_s.size() - 1) - track.vals_s.head(track.vals_s.size() - 1);

    // fit linear splines
    track.coeffs_X.resize(size - 1, 2);
    track.coeffs_Y.resize(size - 1, 2);
    track.coeffs_phi.resize(size - 1, 2);
    track.coeffs_kappa.resize(size - 1, 2);
    track.coeffs_width.resize(size - 1, 2);
    for (size_t i = 0; i < size - 1; ++i) {
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

tl::optional<Track> brains2::common::Track::from_file(const std::filesystem::path& csv_file) {
    rapidcsv::Document doc(csv_file.string());
    return Track::from_values(doc.GetColumn<double>("s"),
                              doc.GetColumn<double>("X"),
                              doc.GetColumn<double>("Y"),
                              doc.GetColumn<double>("phi"),
                              doc.GetColumn<double>("kappa"),
                              doc.GetColumn<double>("w"));
}

double brains2::common::Track::length() const {
    // This class usually represents 'open' tracks. When it is used to represent a closed track, the
    // length will be slightly off as we should add
    //  std::hypot(vals_X(size - 1) - vals_X(0), vals_Y(size - 1) - vals_Y(0))
    return *vals_s.end() - *vals_s.begin();
}
size_t brains2::common::Track::size() const {
    return vals_s.size();
}

static double angle3pt(const Eigen::Vector2d& a,
                       const Eigen::Vector2d& b,
                       const Eigen::Vector2d& c) {
    return brains2::common::wrap_to_pi(std::atan2(c(1) - b(1), c(0) - b(0)) -
                                       std::atan2(a(1) - b(1), a(0) - b(0)));
}

double brains2::common::Track::interp(const Eigen::MatrixXd& coeffs, double s, int ind) const {
    // find i such that s_ref[i] <= s < s_ref[i+1]
    if (ind < 0) {
        ind = find_interval(s);
    }
    // find the value of the spline at s
    return coeffs(ind, 0) + coeffs(ind, 1) * (s - vals_s(ind));
}

std::tuple<double, Eigen::Vector2d> Track::project(const Eigen::Vector2d& car_pos,
                                                   double s_guess,
                                                   double s_tol) const {
    // extract all the points in X_ref, Y_ref associated with s_ref values
    // within s_guess +- s_tol
    double s_low = std::max(s_guess - s_tol, vals_s(0)),
           s_up = std::min(s_guess + s_tol, vals_s(vals_s.size() - 1));
    long long id_low = find_interval(s_low), id_up = find_interval(s_up);
    if (id_low > 0) {
        --id_low;
    }
    if (id_up < static_cast<long long>(vals_s.size()) - 1) {
        ++id_up;
    }
    Eigen::ArrayX2d local_traj =
        Eigen::ArrayX2d::Zero(id_up - id_low + 1,
                              2);  // problem with difference of size_ints ?
    local_traj.col(0) = vals_X.segment(id_low, id_up - id_low + 1);
    local_traj.col(1) = vals_Y.segment(id_low, id_up - id_low + 1);

    // find the closest point to car_pos to find one segment extremity
    Eigen::VectorXd sqdist =
        (local_traj.col(0) - car_pos(0)).square() + (local_traj.col(1) - car_pos(1)).square();
    long long id_min, id_prev, id_next;
    sqdist.minCoeff(&id_min);
    id_prev = id_min - 1;
    if (id_min == 0) {
        id_prev = local_traj.rows() - 1;
    }
    id_next = id_min + 1;
    if (id_min == static_cast<long long>(local_traj.rows()) - 1) {
        id_next = 0;
    }
    // TODO: what happens if id_min == 0 or id_min == local_traj.rows() - 1 ?
    // This should not happen though

    // compute the angles between car_pos, the closest point and the next and
    // previous point to find the second segment extremity
    double angle_prev =
               std::fabs(angle3pt(local_traj.row(id_min), car_pos, local_traj.row(id_prev))),
           angle_next =
               std::fabs(angle3pt(local_traj.row(id_min), car_pos, local_traj.row(id_next)));
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
    double lambda = ((car_pos(0) - a(0)) * dx + (car_pos(1) - a(1)) * dy) / (dx * dx + dy * dy);

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

double Track::eval_track_width(double s) const {
    return this->interp(this->coeffs_width, s);
}

Eigen::Vector2d Track::frenet_to_cartesian(const double& s, const double& n) const {
    double X_ref = this->interp(this->coeffs_X, s);
    double Y_ref = this->interp(this->coeffs_Y, s);
    double phi_ref = this->interp(this->coeffs_phi, s);
    Eigen::Vector2d normal(-std::sin(phi_ref), std::cos(phi_ref));
    return Eigen::Vector2d(X_ref + n * normal(0), Y_ref + n * normal(1));
}

size_t brains2::common::Track::find_interval(double s) const {
    return std::upper_bound(vals_s.data(), vals_s.data() + vals_s.size(), s) - vals_s.data() - 1;
}

const Eigen::VectorXd& brains2::common::Track::get_vals_s() const {
    return this->vals_s;
}

const Eigen::VectorXd& brains2::common::Track::get_vals_X() const {
    return this->vals_X;
}

const Eigen::VectorXd& brains2::common::Track::get_vals_Y() const {
    return this->vals_Y;
}

const Eigen::VectorXd& brains2::common::Track::get_vals_phi() const {
    return this->vals_phi;
}

const Eigen::VectorXd& brains2::common::Track::get_vals_kappa() const {
    return this->vals_kappa;
}

const Eigen::VectorXd& brains2::common::Track::get_vals_width() const {
    return this->vals_width;
}

}  // namespace brains2::common
