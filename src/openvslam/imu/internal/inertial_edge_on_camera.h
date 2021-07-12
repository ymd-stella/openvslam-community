#ifndef OPENVSLAM_IMU_INTERNAL_INERTIAL_EDGE_ON_CAMERA_H
#define OPENVSLAM_IMU_INTERNAL_INERTIAL_EDGE_ON_CAMERA_H

#include "openvslam/type.h"
#include "openvslam/util/converter.h"
#include "openvslam/imu/bias.h"
#include "openvslam/imu/constant.h"
#include "openvslam/optimize/internal/se3/shot_vertex.h"
#include "openvslam/imu/internal/velocity_vertex.h"
#include "openvslam/imu/internal/bias_vertex.h"
#include "openvslam/imu/config.h"

#include <g2o/core/base_multi_edge.h>

namespace openvslam {
namespace imu {
namespace internal {

class inertial_edge_on_camera final : public g2o::BaseMultiEdge<9, std::shared_ptr<preintegrated>> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inertial_edge_on_camera(const std::shared_ptr<imu::config>& cfg);

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void computeError() override;

    void linearizeOplus() override;

    Vec3_t gravity;
    std::shared_ptr<imu::config> cfg_;
};

inline inertial_edge_on_camera::inertial_edge_on_camera(const std::shared_ptr<imu::config>& cfg)
    : g2o::BaseMultiEdge<9, std::shared_ptr<preintegrated>>() {
    resize(6);
    gravity << 0, 0, -imu::constant::gravity();
    cfg_ = cfg;
}

inline bool inertial_edge_on_camera::read(std::istream& is) {
    (void)is;
    return false;
}

inline bool inertial_edge_on_camera::write(std::ostream& os) const {
    (void)os;
    return false;
}

inline void inertial_edge_on_camera::computeError() {
    const auto keyfrm_vtx1 = static_cast<const optimize::internal::se3::shot_vertex*>(_vertices[0]);
    const auto velocity_vtx1 = static_cast<const velocity_vertex*>(_vertices[1]);
    const auto gyr_bias_vtx = static_cast<const bias_vertex*>(_vertices[2]);
    const auto acc_bias_vtx = static_cast<const bias_vertex*>(_vertices[3]);
    const auto keyfrm_vtx2 = static_cast<const optimize::internal::se3::shot_vertex*>(_vertices[4]);
    const auto velocity_vtx2 = static_cast<const velocity_vertex*>(_vertices[5]);

    const bias b(acc_bias_vtx->estimate(), gyr_bias_vtx->estimate());
    const Mat33_t delta_rotation = _measurement->get_delta_rotation_on_bias(b);
    const Vec3_t delta_velocity = _measurement->get_delta_velocity_on_bias(b);
    const Vec3_t delta_position = _measurement->get_delta_position_on_bias(b);
    const double dt = _measurement->dt_;

    const Mat33_t Rcw1 = keyfrm_vtx1->estimate().rotation().toRotationMatrix();
    const Mat33_t Riw1 = cfg_->get_rel_rot_ic() * Rcw1;
    const Mat33_t Rwi1 = Riw1.transpose();
    const Vec3_t tcw1 = keyfrm_vtx1->estimate().translation();
    const Vec3_t twi1 = -Rwi1 * (cfg_->get_rel_rot_ic() * tcw1 + cfg_->get_rel_trans_ic());
    const Mat33_t Rcw2 = keyfrm_vtx2->estimate().rotation().toRotationMatrix();
    const Mat33_t Riw2 = cfg_->get_rel_rot_ic() * Rcw2;
    const Mat33_t Rwi2 = Riw2.transpose();
    const Vec3_t tcw2 = keyfrm_vtx2->estimate().translation();
    const Vec3_t twi2 = -Rwi2 * (cfg_->get_rel_rot_ic() * tcw2 + cfg_->get_rel_trans_ic());

    const Vec3_t v1 = velocity_vtx1->estimate();
    const Vec3_t v2 = velocity_vtx2->estimate();

    const Vec3_t error_rotation = util::converter::log_so3(delta_rotation.transpose() * Riw1 * Rwi2);
    const Vec3_t error_velocity = Riw1 * (v2 - v1 - gravity * dt) - delta_velocity;
    const Vec3_t error_position = Riw1 * (twi2 - twi1 - v1 * dt - 0.5 * gravity * dt * dt) - delta_position;

    _error << error_rotation, error_velocity, error_position;
}

inline void inertial_edge_on_camera::linearizeOplus() {
    const auto keyfrm_vtx1 = static_cast<const optimize::internal::se3::shot_vertex*>(_vertices[0]);
    const auto velocity_vtx1 = static_cast<const velocity_vertex*>(_vertices[1]);
    const auto gyr_bias_vtx = static_cast<const bias_vertex*>(_vertices[2]);
    const auto acc_bias_vtx = static_cast<const bias_vertex*>(_vertices[3]);
    const auto keyfrm_vtx2 = static_cast<const optimize::internal::se3::shot_vertex*>(_vertices[4]);
    const auto velocity_vtx2 = static_cast<const velocity_vertex*>(_vertices[5]);

    const imu::bias b(acc_bias_vtx->estimate(), gyr_bias_vtx->estimate());
    const imu::bias& b0 = _measurement->b_;
    const Vec3_t delta_bias_gyr = b.gyr_ - b0.gyr_;

    const Mat33_t jacob_rotation_gyr = _measurement->jacob_rotation_gyr_;
    const Mat33_t jacob_velocity_gyr = _measurement->jacob_velocity_gyr_;
    const Mat33_t jacob_position_gyr = _measurement->jacob_position_gyr_;
    const Mat33_t jacob_velocity_acc = _measurement->jacob_velocity_acc_;
    const Mat33_t jacob_position_acc = _measurement->jacob_position_acc_;

    const Mat33_t Rcw1 = keyfrm_vtx1->estimate().rotation().toRotationMatrix();
    const Mat33_t Riw1 = cfg_->get_rel_rot_ic() * Rcw1;
    const Mat33_t Rwi1 = Riw1.transpose();
    const Vec3_t tcw1 = keyfrm_vtx1->estimate().translation();
    const Vec3_t twi1 = -Rwi1 * (cfg_->get_rel_rot_ic() * tcw1 + cfg_->get_rel_trans_ic());
    const Mat33_t Rcw2 = keyfrm_vtx2->estimate().rotation().toRotationMatrix();
    const Mat33_t Riw2 = cfg_->get_rel_rot_ic() * Rcw2;
    const Mat33_t Rwi2 = Riw2.transpose();
    const Vec3_t tcw2 = keyfrm_vtx2->estimate().translation();
    const Vec3_t twi2 = -Rwi2 * (cfg_->get_rel_rot_ic() * tcw2 + cfg_->get_rel_trans_ic());

    const Vec3_t v1 = velocity_vtx1->estimate();
    const Vec3_t v2 = velocity_vtx2->estimate();

    const Mat33_t delta_rotation = _measurement->get_delta_rotation_on_bias(b);
    const Mat33_t error_rotation = delta_rotation.transpose() * Riw1 * Rwi2;
    const Mat33_t inv_right_jacobian = util::converter::inverse_right_jacobian_so3(util::converter::log_so3(error_rotation));
    const double dt = _measurement->dt_;

    const Mat33_t rel_rot_ci = cfg_->get_rel_rot_ic().transpose();
    const Mat33_t rel_rot_ic = cfg_->get_rel_rot_ic();
    // Jacobians wrt Pose 1
    _jacobianOplus[0].setZero();
    // rotation
    _jacobianOplus[0].block<3, 3>(0, 0) = -inv_right_jacobian * Riw2 * Rwi1 * rel_rot_ic;
    _jacobianOplus[0].block<3, 3>(3, 0) = util::converter::to_skew_symmetric_mat(Riw1 * (v2 - v1 - gravity * dt)) * rel_rot_ic;
    _jacobianOplus[0].block<3, 3>(6, 0) = util::converter::to_skew_symmetric_mat(Riw1 * (twi2 - twi1 - v1 * dt - 0.5 * gravity * dt * dt)) * rel_rot_ic;
    // translation
    _jacobianOplus[0].block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity() * rel_rot_ic;

    // Jacobians wrt Velocity 1
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3, 3>(3, 0) = -Riw1;
    _jacobianOplus[1].block<3, 3>(6, 0) = -Riw1 * dt;

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
    _jacobianOplus[4].block<3, 3>(0, 3) = inv_right_jacobian * rel_rot_ic;
    // translation
    _jacobianOplus[4].block<3, 3>(6, 0) = Riw1 * Rwi2 * rel_rot_ic;

    // Jacobians wrt Velocity 2
    _jacobianOplus[5].setZero();
    _jacobianOplus[5].block<3, 3>(3, 0) = Riw1;
}

} // namespace internal
} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_INTERNAL_INERTIAL_EDGE_ON_CAMERA_H
