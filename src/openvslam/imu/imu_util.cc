#include "openvslam/imu/imu_util.h"
#include "openvslam/imu/data.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/util/converter.h"
#include "openvslam/imu/preintegrator.h"
#include "openvslam/imu/preintegrated.h"

namespace openvslam {
namespace imu {

void imu_util::preprocess_imu(const imu::data& imu1, const imu::data& imu2,
                              Vec3_t& acc, Vec3_t& gyr, double& dt) {
    // Take the average of two consecutive values for numerical integration with the trapezoidal rule.
    Vec3_t acc1 = imu1.acc_;
    Vec3_t gyr1 = imu1.gyr_;
    Vec3_t acc2 = imu2.acc_;
    Vec3_t gyr2 = imu2.gyr_;
    dt = imu2.ts_ - imu1.ts_;
    acc = (acc1 + acc2) * 0.5;
    gyr = (gyr1 + gyr2) * 0.5;
}

void imu_util::preprocess_imu_interpolate1(const imu::data& imu1, const imu::data& imu2,
                                           double last_stamp, Vec3_t& acc, Vec3_t& gyr, double& dt) {
    double dt1f = last_stamp - imu1.ts_;
    double dt12 = imu2.ts_ - imu1.ts_;
    Vec3_t acc1 = imu2.acc_;
    Vec3_t gyr1 = imu2.gyr_;
    // interpolation
    Vec3_t acc2 = imu1.acc_ + (imu2.acc_ - imu1.acc_) * (dt1f / dt12);
    Vec3_t gyr2 = imu1.gyr_ + (imu2.gyr_ - imu1.gyr_) * (dt1f / dt12);
    dt = dt1f;
    acc = (acc1 + acc2) * 0.5;
    gyr = (gyr1 + gyr2) * 0.5;
}

void imu_util::preprocess_imu_interpolate2(const imu::data& imu1, const imu::data& imu2,
                                           double curr_stamp, Vec3_t& acc, Vec3_t& gyr, double& dt) {
    double dt1f = imu2.ts_ - curr_stamp;
    double dt12 = imu2.ts_ - imu1.ts_;
    Vec3_t acc1 = imu1.acc_;
    Vec3_t gyr1 = imu1.gyr_;
    // interpolation
    Vec3_t acc2 = imu1.acc_ + (imu2.acc_ - imu1.acc_) * (dt1f / dt12);
    Vec3_t gyr2 = imu1.gyr_ + (imu2.gyr_ - imu1.gyr_) * (dt1f / dt12);
    dt = dt1f;
    acc = (acc1 + acc2) * 0.5;
    gyr = (gyr1 + gyr2) * 0.5;
}

std::vector<openvslam::data::keyframe*> imu_util::gather_intertial_ref_keyframes(openvslam::data::keyframe* keyfrm) {
    std::vector<openvslam::data::keyframe*> keyfrms;
    keyfrms.push_back(keyfrm);
    while (keyfrms.back()->inertial_ref_keyfrm_) {
        keyfrms.push_back(keyfrms.back()->inertial_ref_keyfrm_);
    }
    return keyfrms;
}

Vec3_t imu_util::compute_velocity(const openvslam::data::keyframe* keyfrm) {
    const Mat44_t pose1_wi = keyfrm->inertial_ref_keyfrm_->get_imu_pose_inv();
    const Vec3_t twi1 = pose1_wi.block<3, 1>(0, 3);
    const Mat44_t pose2_wi = keyfrm->get_imu_pose_inv();
    const Vec3_t twi2 = pose2_wi.block<3, 1>(0, 3);
    return (twi2 - twi1) / keyfrm->imu_preintegrator_from_inertial_ref_keyfrm_->preintegrated_->dt_;
}

Vec3_t imu_util::compute_velocity_interpolate(const openvslam::data::keyframe* keyfrm) {
    Vec3_t vel2 = compute_velocity(keyfrm->inertial_referrer_keyfrm_);
    Vec3_t vel1 = compute_velocity(keyfrm);
    double dt2 = keyfrm->inertial_referrer_keyfrm_->imu_preintegrator_from_inertial_ref_keyfrm_->preintegrated_->dt_;
    double dt1 = keyfrm->imu_preintegrator_from_inertial_ref_keyfrm_->preintegrated_->dt_;
    return vel1 + (vel2 - vel1) * dt1 / (dt1 + dt2);
}

Vec3_t imu_util::compute_velocity_extrapolate1(const openvslam::data::keyframe* keyfrm) {
    Vec3_t vel2 = compute_velocity(keyfrm);
    Vec3_t vel1 = compute_velocity(keyfrm->inertial_ref_keyfrm_);
    double dt2 = keyfrm->imu_preintegrator_from_inertial_ref_keyfrm_->preintegrated_->dt_;
    double dt1 = keyfrm->inertial_ref_keyfrm_->imu_preintegrator_from_inertial_ref_keyfrm_->preintegrated_->dt_;
    return vel1 - (vel2 - vel1) * dt1 / (dt1 + dt2);
}

Vec3_t imu_util::compute_velocity_extrapolate2(const openvslam::data::keyframe* keyfrm) {
    Vec3_t vel2 = compute_velocity(keyfrm);
    Vec3_t vel1 = compute_velocity(keyfrm->inertial_ref_keyfrm_);
    double dt2 = keyfrm->imu_preintegrator_from_inertial_ref_keyfrm_->preintegrated_->dt_;
    double dt1 = keyfrm->inertial_ref_keyfrm_->imu_preintegrator_from_inertial_ref_keyfrm_->preintegrated_->dt_;
    return vel2 + (vel2 - vel1) * dt2 / (dt1 + dt2);
}

void imu_util::compute_velocity(std::vector<openvslam::data::keyframe*>& keyfrms) {
    for (auto iter = keyfrms.cbegin() + 1; iter != keyfrms.cend() - 1; ++iter) {
        auto keyfrm = *iter;
        keyfrm->velocity_ = compute_velocity_interpolate(keyfrm);
    }
    if (keyfrms.size() > 2) {
        keyfrms.back()->velocity_ = compute_velocity_extrapolate1(*(keyfrms.cend() - 3));
        keyfrms[0]->velocity_ = compute_velocity_extrapolate2(keyfrms[0]);
    }
}

Vec3_t imu_util::compute_gravity(openvslam::data::keyframe* keyfrm) {
    const Mat44_t pose1_wi = keyfrm->inertial_ref_keyfrm_->get_imu_pose_inv();
    const Mat33_t Rwi1 = pose1_wi.block<3, 3>(0, 0);
    return -Rwi1 * keyfrm->imu_preintegrator_from_inertial_ref_keyfrm_->preintegrated_->delta_velocity_;
}

Mat33_t imu_util::compute_gravity_dir(std::vector<openvslam::data::keyframe*>& keyfrms) {
    Vec3_t integrated_gravity = Vec3_t::Zero();
    for (auto iter = keyfrms.cbegin(); iter != keyfrms.cend() - 1; ++iter) {
        auto keyfrm = *iter;
        const Mat44_t pose1_wi = keyfrm->inertial_ref_keyfrm_->get_imu_pose_inv();
        const Mat33_t Rwi1 = pose1_wi.block<3, 3>(0, 0);
        integrated_gravity += compute_gravity(keyfrm);
    }

    integrated_gravity = integrated_gravity / integrated_gravity.norm();
    const Vec3_t ez(0.0, 0.0, -1.0);
    const Vec3_t normal = ez.cross(integrated_gravity);
    const Vec3_t dir_normal = normal / normal.norm();
    const double cos_gravity = ez.dot(integrated_gravity);
    const Vec3_t rot_wg = dir_normal * std::acos(cos_gravity);
    return util::converter::exp_so3(rot_wg);
}

} // namespace imu
} // namespace openvslam
