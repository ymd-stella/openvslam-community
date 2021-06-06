#include "openvslam/util/converter.h"

namespace {
template<typename T>
T clamp(const T& n, const T& lower, const T& upper) {
    return std::max(lower, std::min(n, upper));
}
} // namespace

namespace openvslam {
namespace util {

std::vector<cv::Mat> converter::to_desc_vec(const cv::Mat& desc) {
    std::vector<cv::Mat> desc_vec(desc.rows);
    for (int i = 0; i < desc.rows; ++i) {
        desc_vec.at(i) = desc.row(i);
    }
    return desc_vec;
}

g2o::SE3Quat converter::to_g2o_SE3(const Mat44_t& cam_pose) {
    const Mat33_t rot = cam_pose.block<3, 3>(0, 0);
    const Vec3_t trans = cam_pose.block<3, 1>(0, 3);
    return g2o::SE3Quat{rot, trans};
}

Mat44_t converter::to_eigen_mat(const g2o::SE3Quat& g2o_SE3) {
    return g2o_SE3.to_homogeneous_matrix();
}

Mat44_t converter::to_eigen_mat(const g2o::Sim3& g2o_Sim3) {
    Mat44_t cam_pose = Mat44_t::Identity();
    cam_pose.block<3, 3>(0, 0) = g2o_Sim3.scale() * g2o_Sim3.rotation().toRotationMatrix();
    cam_pose.block<3, 1>(0, 3) = g2o_Sim3.translation();
    return cam_pose;
}

Mat44_t converter::to_eigen_cam_pose(const Mat33_t& rot, const Vec3_t& trans) {
    Mat44_t cam_pose = Mat44_t::Identity();
    cam_pose.block<3, 3>(0, 0) = rot;
    cam_pose.block<3, 1>(0, 3) = trans;
    return cam_pose;
}

Vec3_t converter::to_angle_axis(const Mat33_t& rot_mat) {
    const Eigen::AngleAxisd angle_axis(rot_mat);
    return angle_axis.axis() * angle_axis.angle();
}

Mat33_t converter::to_rot_mat(const Vec3_t& angle_axis) {
    Eigen::Matrix3d rot_mat;
    const double angle = angle_axis.norm();
    if (angle <= 1e-5) {
        rot_mat = Eigen::Matrix3d::Identity();
    }
    else {
        const Eigen::Vector3d axis = angle_axis / angle;
        rot_mat = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    }
    return rot_mat;
}

Mat33_t converter::to_skew_symmetric_mat(const Vec3_t& vec) {
    Mat33_t skew;
    skew << 0, -vec(2), vec(1),
        vec(2), 0, -vec(0),
        -vec(1), vec(0), 0;
    return skew;
}

Mat33_t converter::normalize_rotation(const Mat33_t& R) {
    Eigen::Quaterniond q(R);
    return q.normalized().toRotationMatrix();
}

Vec3_t converter::vee(const Mat33_t& R) {
    return Vec3_t(R(2, 1), R(0, 2), R(1, 0));
}

Mat33_t converter::exp_so3(const Vec3_t& v) {
    // The reference is "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", (3)
    const Mat33_t I = Mat33_t::Identity();
    const double d_sq = v.squaredNorm();
    const double d = sqrt(d_sq);
    const Mat33_t W = to_skew_symmetric_mat(v);
    const double eps = 1e-4;
    if (d < eps) {
        return I + W + 0.5 * W * W;
    }
    else {
        return I + W * std::sin(d) / d + W * W * (1.0 - std::cos(d)) / d_sq;
    }
}

Vec3_t converter::log_so3(const Mat33_t& R) {
    const double theta = std::acos(clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0));
    const double sin_theta = std::sin(theta);
    const double eps = 1e-4;
    if (std::abs(sin_theta) < eps) {
        return vee(R - R.transpose()) / 2.0;
    }
    else {
        return vee(R - R.transpose()) * theta / (2.0 * sin_theta);
    }
}

Mat33_t converter::inverse_right_jacobian_so3(const Vec3_t& v) {
    const Mat33_t I = Mat33_t::Identity();
    const double d_sq = v.squaredNorm();
    const double d = std::sqrt(d_sq);
    const Mat33_t W = to_skew_symmetric_mat(v);
    const double eps = 1e-4;
    if (d < eps) {
        return I;
    }
    else {
        return I + W / 2.0 + W * W * (1.0 / d_sq - (1.0 + std::cos(d)) / (2.0 * d * std::sin(d)));
    }
}

Mat33_t converter::right_jacobian_so3(const Vec3_t& v) {
    // The reference is "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", (8)
    const Mat33_t I = Mat33_t::Identity();
    const double d_sq = v.squaredNorm();
    const double d = std::sqrt(d_sq);
    const Mat33_t W = to_skew_symmetric_mat(v);
    const double eps = 1e-4;
    if (d < eps) {
        return I;
    }
    else {
        return I - W * (1.0 - std::cos(d)) / d_sq + W * W * (d - std::sin(d)) / (d_sq * d);
    }
}

} // namespace util
} // namespace openvslam
