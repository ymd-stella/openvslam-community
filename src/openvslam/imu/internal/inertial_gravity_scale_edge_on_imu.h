#ifndef OPENVSLAM_IMU_INTERNAL_GRAVITY_SCALE_EDGE_ON_IMU_H
#define OPENVSLAM_IMU_INTERNAL_GRAVITY_SCALE_EDGE_ON_IMU_H

#include "openvslam/type.h"
#include "openvslam/util/converter.h"
#include "openvslam/imu/bias.h"
#include "openvslam/imu/constant.h"
#include "openvslam/imu/config.h"
#include "openvslam/imu/preintegrated.h"
#include "openvslam/optimize/internal/se3/shot_vertex.h"
#include "openvslam/imu/internal/velocity_vertex.h"
#include "openvslam/imu/internal/bias_vertex.h"
#include "openvslam/imu/internal/gravity_dir_vertex.h"
#include "openvslam/imu/internal/scale_vertex.h"

#include <g2o/core/base_multi_edge.h>

namespace openvslam {
namespace imu {
namespace internal {

class inertial_gravity_scale_edge_on_imu final : public g2o::BaseMultiEdge<9, std::shared_ptr<preintegrated>> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inertial_gravity_scale_edge_on_imu();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void computeError() override;

    void linearizeOplus() override;
    Vec3_t gravity;
};

inline inertial_gravity_scale_edge_on_imu::inertial_gravity_scale_edge_on_imu()
    : g2o::BaseMultiEdge<9, std::shared_ptr<preintegrated>>() {
    resize(8);
    gravity << 0, 0, -imu::constant::gravity();
}

inline bool inertial_gravity_scale_edge_on_imu::read(std::istream& is) {
    double dt;
    MatRC_t<15, 15> covariance;
    Vec3_t acc;
    Vec3_t gyr;
    Mat33_t delta_rotation;
    Vec3_t delta_velocity;
    Vec3_t delta_position;
    Mat33_t jacob_rotation_gyr;
    Mat33_t jacob_velocity_gyr;
    Mat33_t jacob_velocity_acc;
    Mat33_t jacob_position_gyr;
    Mat33_t jacob_position_acc;
    is >> dt;
    read_matrix(is, covariance);
    read_matrix(is, acc);
    read_matrix(is, gyr);
    read_matrix(is, delta_rotation);
    read_matrix(is, delta_velocity);
    read_matrix(is, delta_position);
    read_matrix(is, jacob_rotation_gyr);
    read_matrix(is, jacob_velocity_gyr);
    read_matrix(is, jacob_velocity_acc);
    read_matrix(is, jacob_position_gyr);
    read_matrix(is, jacob_position_acc);
    _measurement = eigen_alloc_shared<preintegrated>(dt, covariance, bias(acc, gyr), delta_rotation, delta_velocity, delta_position,
                                                     jacob_rotation_gyr, jacob_velocity_gyr, jacob_velocity_acc, jacob_position_gyr, jacob_position_acc);
    for (unsigned int i = 0; i < Dimension; ++i) {
        for (unsigned int j = i; j < Dimension; ++j) {
            is >> information()(i, j);
            if (i != j) {
                information()(j, i) = information()(i, j);
            }
        }
    }
    return true;
}

inline bool inertial_gravity_scale_edge_on_imu::write(std::ostream& os) const {
    os << _measurement->dt_ << " ";
    write_matrix(os, _measurement->covariance_);
    write_matrix(os, _measurement->b_.acc_);
    write_matrix(os, _measurement->b_.gyr_);
    write_matrix(os, _measurement->delta_rotation_);
    write_matrix(os, _measurement->delta_velocity_);
    write_matrix(os, _measurement->delta_position_);
    write_matrix(os, _measurement->jacob_rotation_gyr_);
    write_matrix(os, _measurement->jacob_velocity_gyr_);
    write_matrix(os, _measurement->jacob_velocity_acc_);
    write_matrix(os, _measurement->jacob_position_gyr_);
    write_matrix(os, _measurement->jacob_position_acc_);
    for (unsigned int i = 0; i < Dimension; ++i) {
        for (unsigned int j = i; j < Dimension; ++j) {
            os << " " << information()(i, j);
        }
    }
    return os.good();
}

inline void inertial_gravity_scale_edge_on_imu::computeError() {
    const auto keyfrm_vtx1 = static_cast<const optimize::internal::se3::shot_vertex*>(_vertices[0]);
    const auto velocity_vtx1 = static_cast<const velocity_vertex*>(_vertices[1]);
    const auto gyr_bias_vtx = static_cast<const bias_vertex*>(_vertices[2]);
    const auto acc_bias_vtx = static_cast<const bias_vertex*>(_vertices[3]);
    const auto keyfrm_vtx2 = static_cast<const optimize::internal::se3::shot_vertex*>(_vertices[4]);
    const auto velocity_vtx2 = static_cast<const velocity_vertex*>(_vertices[5]);
    const auto gravity_dir_vtx = static_cast<const gravity_dir_vertex*>(_vertices[6]);
    const auto scale_vtx = static_cast<const scale_vertex*>(_vertices[7]);

    const bias b(acc_bias_vtx->estimate(), gyr_bias_vtx->estimate());
    const Mat33_t delta_rotation = _measurement->get_delta_rotation_on_bias(b);
    const Vec3_t delta_velocity = _measurement->get_delta_velocity_on_bias(b);
    const Vec3_t delta_position = _measurement->get_delta_position_on_bias(b);
    const double dt = _measurement->dt_;

    const Mat33_t Riw1 = keyfrm_vtx1->estimate().rotation().toRotationMatrix();
    const Mat33_t Rwi1 = Riw1.transpose();
    const Vec3_t twi1 = -Rwi1 * keyfrm_vtx1->estimate().translation();
    const Mat33_t Riw2 = keyfrm_vtx2->estimate().rotation().toRotationMatrix();
    const Mat33_t Rwi2 = Riw2.transpose();
    const Vec3_t twi2 = -Rwi2 * keyfrm_vtx2->estimate().translation();
    const Vec3_t v1 = velocity_vtx1->estimate();
    const Vec3_t v2 = velocity_vtx2->estimate();

    const Vec3_t g = gravity_dir_vtx->estimate() * gravity;
    const double s = scale_vtx->estimate();

    // The reference is "Inertial-Only Optimization for Visual-Inertial Initialization"
    const Vec3_t error_rotation = util::converter::log_so3(delta_rotation.transpose() * Riw1 * Rwi2);        // (7)
    const Vec3_t error_velocity = Riw1 * (s * (v2 - v1) - g * dt) - delta_velocity;                          // (8)
    const Vec3_t error_position = Riw1 * (s * (twi2 - twi1 - v1 * dt) - 0.5 * g * dt * dt) - delta_position; // (9)

    _error << error_rotation, error_velocity, error_position;
}

inline void inertial_gravity_scale_edge_on_imu::linearizeOplus() {
    const auto keyfrm_vtx1 = static_cast<const optimize::internal::se3::shot_vertex*>(_vertices[0]);
    const auto velocity_vtx1 = static_cast<const velocity_vertex*>(_vertices[1]);
    const auto gyr_bias_vtx = static_cast<const bias_vertex*>(_vertices[2]);
    const auto acc_bias_vtx = static_cast<const bias_vertex*>(_vertices[3]);
    const auto keyfrm_vtx2 = static_cast<const optimize::internal::se3::shot_vertex*>(_vertices[4]);
    const auto velocity_vtx2 = static_cast<const velocity_vertex*>(_vertices[5]);
    const auto gravity_dir_vtx = static_cast<const gravity_dir_vertex*>(_vertices[6]);
    const auto scale_vtx = static_cast<const scale_vertex*>(_vertices[7]);

    const imu::bias b(acc_bias_vtx->estimate(), gyr_bias_vtx->estimate());
    const imu::bias& b0 = _measurement->b_;
    const Vec3_t delta_bias_gyr = b.gyr_ - b0.gyr_;

    const Mat33_t jacob_rotation_gyr = _measurement->jacob_rotation_gyr_;
    const Mat33_t jacob_velocity_gyr = _measurement->jacob_velocity_gyr_;
    const Mat33_t jacob_position_gyr = _measurement->jacob_position_gyr_;
    const Mat33_t jacob_velocity_acc = _measurement->jacob_velocity_acc_;
    const Mat33_t jacob_position_acc = _measurement->jacob_position_acc_;

    const Mat33_t Riw1 = keyfrm_vtx1->estimate().rotation().toRotationMatrix();
    const Mat33_t Rwi1 = Riw1.transpose();
    const Vec3_t twi1 = -Rwi1 * keyfrm_vtx1->estimate().translation();
    const Mat33_t Riw2 = keyfrm_vtx2->estimate().rotation().toRotationMatrix();
    const Mat33_t Rwi2 = Riw2.transpose();
    const Vec3_t twi2 = -Rwi2 * keyfrm_vtx2->estimate().translation();

    const Mat33_t Rwg = gravity_dir_vtx->estimate();
    MatRC_t<3, 2> Gm;
    Gm << 0.0, -imu::constant::gravity(),
        imu::constant::gravity(), 0.0,
        0.0, 0.0;
    const MatRC_t<3, 2> dGdTheta = Rwg * Gm;

    const Vec3_t g = Rwg * gravity;
    const double s = scale_vtx->estimate();

    const Vec3_t v1 = velocity_vtx1->estimate();
    const Vec3_t v2 = velocity_vtx2->estimate();

    const Mat33_t delta_rotation = _measurement->get_delta_rotation_on_bias(b);
    const Mat33_t error_rotation = delta_rotation.transpose() * Riw1 * Rwi2;
    const Mat33_t inv_right_jacobian = util::converter::inverse_right_jacobian_so3(util::converter::log_so3(error_rotation));
    const double dt = _measurement->dt_;

    // The reference is "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", Appendix C
    // Jacobians wrt Pose 1
    _jacobianOplus[0].setZero();
    // rotation
    _jacobianOplus[0].block<3, 3>(0, 0) = -inv_right_jacobian * Riw2 * Rwi1;
    _jacobianOplus[0].block<3, 3>(3, 0) = util::converter::to_skew_symmetric_mat(Riw1 * (s * (v2 - v1) - g * dt));
    _jacobianOplus[0].block<3, 3>(6, 0) = util::converter::to_skew_symmetric_mat(Riw1 * (s * (twi2 - twi1 - v1 * dt) - 0.5 * g * dt * dt));
    // translation
    _jacobianOplus[0].block<3, 3>(6, 3) = -s * Eigen::Matrix3d::Identity();

    // Jacobians wrt Velocity 1
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3, 3>(3, 0) = -s * Riw1;
    _jacobianOplus[1].block<3, 3>(6, 0) = -s * Riw1 * dt;

    // Jacobians wrt Gyro 1
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3, 3>(0, 0) = -inv_right_jacobian * error_rotation.transpose()
                                          * util::converter::right_jacobian_so3(jacob_rotation_gyr * delta_bias_gyr) * jacob_rotation_gyr;
    _jacobianOplus[2].block<3, 3>(3, 0) = -jacob_velocity_gyr;
    _jacobianOplus[2].block<3, 3>(6, 0) = -jacob_position_gyr;

    // Jacobians wrt Accelerometer 1
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3, 3>(3, 0) = -jacob_velocity_acc;
    _jacobianOplus[3].block<3, 3>(6, 0) = -jacob_position_acc;

    // Jacobians wrt Pose 2
    _jacobianOplus[4].setZero();
    // rotation
    _jacobianOplus[4].block<3, 3>(0, 0) = inv_right_jacobian;
    // translation
    _jacobianOplus[4].block<3, 3>(6, 3) = s * Riw1 * Rwi2;

    // Jacobians wrt Velocity 2
    _jacobianOplus[5].setZero();
    _jacobianOplus[5].block<3, 3>(3, 0) = s * Riw1;

    // The reference is "Inertial-Only Optimization for Visual-Inertial Initialization"
    // Jacobians wrt Gravity direction
    _jacobianOplus[6].setZero();
    _jacobianOplus[6].block<3, 2>(3, 0) = -Riw1 * dGdTheta * dt;            // (17)
    _jacobianOplus[6].block<3, 2>(6, 0) = -0.5 * Riw1 * dGdTheta * dt * dt; // (18)

    // Jacobians wrt scale factor
    _jacobianOplus[7].setZero();
    _jacobianOplus[7].block<3, 1>(3, 0) = Riw1 * (v2 - v1);               // (14)
    _jacobianOplus[7].block<3, 1>(6, 0) = Riw1 * (twi2 - twi1 - v1 * dt); // (15)
}

} // namespace internal
} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_INTERNAL_GRAVITY_SCALE_EDGE_ON_IMU_H
