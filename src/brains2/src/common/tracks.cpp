// Copyright (c) 2024. Tudor Oancea, Matteo Berthet
#include "brains2/common/tracks.hpp"
#include <cmath>
#include <vector>
#include "brains2/common/math.hpp"
#include "brains2/external/icecream.hpp"
#include "brains2/external/rapidcsv.hpp"


brains2::common::Track::Track(const std::vector<double>& s_ref,
                              const std::vector<double>& X_ref,
                              const std::vector<double>& Y_ref,
                              const std::vector<double>& phi_ref,
                              const std::vector<double>& kappa_ref,
                              const std::vector<double>& right_width,
                              const std::vector<double>& left_width) {
    // Check that all the vectors have the same size
    if (s_ref.size() != X_ref.size() || s_ref.size() != Y_ref.size() ||
        s_ref.size() != phi_ref.size() || s_ref.size() != kappa_ref.size() ||
        s_ref.size() != right_width.size() || s_ref.size() != left_width.size()) {
        throw std::invalid_argument("All vectors must have the same size");
    }
    // Copy the data in the Eigen vectors
    this->s_ref = Eigen::VectorXd::Map(s_ref.data(), s_ref.size());
    this->X_ref = Eigen::VectorXd::Map(X_ref.data(), X_ref.size());
    this->Y_ref = Eigen::VectorXd::Map(Y_ref.data(), Y_ref.size());
    this->phi_ref = Eigen::VectorXd::Map(phi_ref.data(), phi_ref.size());
    this->kappa_ref = Eigen::VectorXd::Map(kappa_ref.data(), kappa_ref.size());
    this->right_width = Eigen::VectorXd::Map(right_width.data(), right_width.size());
    this->left_width = Eigen::VectorXd::Map(left_width.data(), left_width.size());
    IC(this->s_ref.data(), s_ref.data());
}

brains2::common::Track::Track(const std::string& csv_file) {
    rapidcsv::Document doc(csv_file);
    auto row_count = static_cast<long long>(doc.GetRowCount());
    s_ref.resize(row_count);
    X_ref.resize(row_count);
    Y_ref.resize(row_count);
    phi_ref.resize(row_count);
    kappa_ref.resize(row_count);
    right_width.resize(row_count);
    left_width.resize(row_count);
    for (long long i = 0; i < row_count; ++i) {
        s_ref(i) = doc.GetCell<double>("s_ref", i);
        X_ref(i) = doc.GetCell<double>("X_ref", i);
        Y_ref(i) = doc.GetCell<double>("Y_ref", i);
        phi_ref(i) = doc.GetCell<double>("phi_ref", i);
        kappa_ref(i) = doc.GetCell<double>("kappa_ref", i);
        right_width(i) = doc.GetCell<double>("right_width", i);
        left_width(i) = doc.GetCell<double>("left_width", i);
    }

    delta_s = s_ref.tail(s_ref.size() - 1) - s_ref.head(s_ref.size() - 1);

    // fit linear splines
    coeffs_X.resize(row_count - 1, 2);
    coeffs_Y.resize(row_count - 1, 2);
    coeffs_phi.resize(row_count - 1, 2);
    coeffs_kappa.resize(row_count - 1, 2);
    coeffs_right_width.resize(row_count - 1, 2);
    coeffs_left_width.resize(row_count - 1, 2);

    for (long long i = 0; i < row_count - 1; ++i) {
        coeffs_X(i, 0) = X_ref(i);
        coeffs_X(i, 1) = (X_ref(i + 1) - X_ref(i)) / delta_s(i);

        coeffs_Y(i, 0) = Y_ref(i);
        coeffs_Y(i, 1) = (Y_ref(i + 1) - Y_ref(i)) / delta_s(i);

        coeffs_phi(i, 0) = phi_ref(i);
        coeffs_phi(i, 1) = (phi_ref(i + 1) - phi_ref(i)) / delta_s(i);

        coeffs_kappa(i, 0) = kappa_ref(i);
        coeffs_kappa(i, 1) = (kappa_ref(i + 1) - kappa_ref(i)) / delta_s(i);

        coeffs_right_width(i, 0) = right_width(i);
        coeffs_right_width(i, 1) = (right_width(i + 1) - right_width(i)) / delta_s(i);

        coeffs_left_width(i, 0) = left_width(i);
        coeffs_left_width(i, 1) = (left_width(i + 1) - left_width(i)) / delta_s(i);
    }
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

void brains2::common::Track::interp(const Eigen::MatrixXd& coeffs,
                                    double s,
                                    double& value,
                                    int ind) const {
    // find i such that s_ref[i] <= s < s_ref[i+1]
    if (ind < 0) {
        ind = locate_index(s_ref, s);
    }
    // find the value of the spline at s
    value = coeffs(ind, 0) + coeffs(ind, 1) * (s - s_ref(ind));
}

void brains2::common::Track::project(const Eigen::Vector2d& car_pos,
                                     double s_guess,
                                     double s_tol,
                                     double* s_proj,
                                     double* X_ref_proj,
                                     double* Y_ref_proj,
                                     double* phi_ref_proj,
                                     double* phi_ref_preview,
                                     double* kappa_ref_proj,
                                     double* right_width_proj,
                                     double* left_width_proj) {
    // extract all the points in X_ref, Y_ref associated with s_ref values
    // within s_guess +- s_tol
    double s_low = std::max(s_guess - s_tol, s_ref(0)),
           s_up = std::min(s_guess + s_tol, s_ref(s_ref.size() - 1));
    long long id_low = locate_index(s_ref, s_low), id_up = locate_index(s_ref, s_up);
    if (id_low > 0) {
        --id_low;
    }
    if (id_up < static_cast<long long>(s_ref.size()) - 1) {
        ++id_up;
    }
    Eigen::ArrayX2d local_traj =
        Eigen::ArrayX2d::Zero(id_up - id_low + 1,
                              2);  // problem with difference of size_ints ?
    local_traj.col(0) = X_ref.segment(id_low, id_up - id_low + 1);
    local_traj.col(1) = Y_ref.segment(id_low, id_up - id_low + 1);

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
        sa = s_ref(id_prev + id_low);
        sb = s_ref(id_min + id_low);
    } else {
        a = local_traj.row(id_min);
        b = local_traj.row(id_next);
        sa = s_ref(id_min + id_low);
        sb = s_ref(id_next + id_low);
    }

    // project car_pos on the segment and retrieve lambda
    double dx = b(0) - a(0), dy = b(1) - a(1);
    double lambda = ((car_pos(0) - a(0)) * dx + (car_pos(1) - a(1)) * dy) / (dx * dx + dy * dy);

    // compute the interpolated values (with non null pointers) at lambda using
    // the index of the closest point
    if (s_proj != nullptr) {
        *s_proj = sa + lambda * (sb - sa);
    }
    if (X_ref_proj != nullptr) {
        *X_ref_proj = a(0) + lambda * (b(0) - a(0));
    }
    if (Y_ref_proj != nullptr) {
        *Y_ref_proj = a(1) + lambda * (b(1) - a(1));
    }
    if (phi_ref_proj != nullptr) {
        if (phi_ref_preview != nullptr) {
            interp(coeffs_phi, *s_proj + *phi_ref_preview, *phi_ref_proj, id_min + id_low);
        } else {
            interp(coeffs_phi, *s_proj, *phi_ref_proj, id_min + id_low);
        }
    }
    if (kappa_ref_proj != nullptr) {
        interp(coeffs_kappa, *s_proj, *kappa_ref_proj, id_min + id_low);
    }
    if (right_width_proj != nullptr) {
        interp(coeffs_right_width, *s_proj, *right_width_proj, id_min + id_low);
    }
    if (left_width_proj != nullptr) {
        interp(coeffs_left_width, *s_proj, *left_width_proj, id_min + id_low);
    }
}

void brains2::common::Track::frenet_to_cartesian(const double& s,
                                                 const double& n,
                                                 double& X,
                                                 double& Y) const {
    double X_ref, Y_ref, phi_ref;
    this->interp(coeffs_phi, s, phi_ref);
    this->interp(coeffs_X, s, X_ref);
    this->interp(coeffs_Y, s, Y_ref);
    Eigen::Vector2d normal(-std::sin(phi_ref), std::cos(phi_ref));
    X = X_ref + n * normal(0);
    Y = Y_ref + n * normal(1);
}

void brains2::common::Track::frenet_to_cartesian(const Eigen::VectorXd& s,
                                                 const Eigen::VectorXd& n,
                                                 Eigen::VectorXd& X,
                                                 Eigen::VectorXd& Y) const {
    X.resize(s.size());
    Y.resize(s.size());
    for (Eigen::Index i = 0; i < s.size(); ++i) {
        this->frenet_to_cartesian(s(i), n(i), X(i), Y(i));
    }
}

double brains2::common::Track::length() const {
    return -s_ref(0);
}
size_t brains2::common::Track::size() const {
    return s_ref.size();
}
double* brains2::common::Track::get_s_ref() {
    return s_ref.data();
}
double* brains2::common::Track::get_kappa_ref() {
    return kappa_ref.data();
}
double* brains2::common::Track::get_X_ref() {
    return X_ref.data();
}
double* brains2::common::Track::get_Y_ref() {
    return Y_ref.data();
}
