// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include "brains2/common/tracks.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <stdexcept>
#include <vector>
#include "brains2/common/math.hpp"
#include "brains2/external/rapidcsv.hpp"

namespace brains2::common {

Track::Track(const std::vector<double>& s,
             const std::vector<double>& X,
             const std::vector<double>& Y,
             const std::vector<double>& phi,
             const std::vector<double>& kappa,
             const std::vector<double>& width) {
    // Check that all the vectors have the same size
    const size_t size = s.size();
    if (size != X.size() || size != Y.size() || size != phi.size() || size != kappa.size() ||
        size != width.size()) {
        throw std::invalid_argument("All vectors must have the same size");
    }
    // Copy the data in the Eigen vectors
    // (the Eigen::Map object creates a view to the data in the std::vector,
    //  but assigning it to an Eigen::Vector copies it)
    this->vals_s = Eigen::VectorXd::Map(s.data(), s.size());
    this->vals_X = Eigen::VectorXd::Map(X.data(), X.size());
    this->vals_Y = Eigen::VectorXd::Map(Y.data(), Y.size());
    this->vals_phi = Eigen::VectorXd::Map(phi.data(), phi.size());
    this->vals_kappa = Eigen::VectorXd::Map(kappa.data(), kappa.size());
    this->vals_width = Eigen::VectorXd::Map(width.data(), width.size());

    // Compute the differences in progress s
    auto delta_s =
        this->vals_s.tail(this->vals_s.size() - 1) - this->vals_s.head(this->vals_s.size() - 1);

    // fit linear splines
    this->coeffs_X.resize(size - 1, 2);
    this->coeffs_Y.resize(size - 1, 2);
    this->coeffs_phi.resize(size - 1, 2);
    this->coeffs_kappa.resize(size - 1, 2);
    this->coeffs_width.resize(size - 1, 2);
    for (size_t i = 0; i < size - 1; ++i) {
        this->coeffs_X(i, 0) = this->vals_X(i);
        this->coeffs_X(i, 1) = (this->vals_X(i + 1) - this->vals_X(i)) / delta_s(i);

        this->coeffs_Y(i, 0) = this->vals_Y(i);
        this->coeffs_Y(i, 1) = (this->vals_Y(i + 1) - this->vals_Y(i)) / delta_s(i);

        this->coeffs_phi(i, 0) = this->vals_phi(i);
        this->coeffs_phi(i, 1) = (this->vals_phi(i + 1) - this->vals_phi(i)) / delta_s(i);

        this->coeffs_kappa(i, 0) = this->vals_kappa(i);
        this->coeffs_kappa(i, 1) = (this->vals_kappa(i + 1) - this->vals_kappa(i)) / delta_s(i);

        this->coeffs_width(i, 0) = this->vals_width(i);
        this->coeffs_width(i, 1) = (this->vals_width(i + 1) - this->vals_width(i)) / delta_s(i);
    }
}

brains2::common::Track::Track(const std::filesystem::path& csv_file) {
    rapidcsv::Document doc(csv_file.string());
    auto row_count = static_cast<long long>(doc.GetRowCount());
    vals_s.resize(row_count);
    vals_X.resize(row_count);
    vals_Y.resize(row_count);
    vals_phi.resize(row_count);
    vals_kappa.resize(row_count);
    vals_width.resize(row_count);
    for (long long i = 0; i < row_count; ++i) {
        vals_s(i) = doc.GetCell<double>("s_ref", i);
        vals_X(i) = doc.GetCell<double>("X_ref", i);
        vals_Y(i) = doc.GetCell<double>("Y_ref", i);
        vals_phi(i) = doc.GetCell<double>("phi_ref", i);
        vals_kappa(i) = doc.GetCell<double>("kappa_ref", i);
        vals_width(i) = doc.GetCell<double>("track_width", i);
    }
}

double brains2::common::Track::length() const {
    return -vals_s(0);
}
size_t brains2::common::Track::size() const {
    return vals_s.size();
}

static size_t locate_index(const Eigen::VectorXd& v, double x) {
    return std::upper_bound(v.data(), v.data() + v.size(), x) - v.data() - 1;
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
        ind = locate_index(vals_s, s);
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
    long long id_low = locate_index(vals_s, s_low), id_up = locate_index(vals_s, s_up);
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

}  // namespace brains2::common
