#ifndef OPENVSLAM_IMU_UTIL_H
#define OPENVSLAM_IMU_UTIL_H

#include <vector>

#include "openvslam/type.h"

namespace openvslam {
namespace data {
class keyframe;
} // namespace data

namespace imu {
class data;

class imu_util {
public:
    //! Computes the value of the IMU between frames
    static void preprocess_imu(const imu::data& imu1, const imu::data& imu2,
                               Vec3_t& acc, Vec3_t& gyr, double& dt);
    static void preprocess_imu_interpolate1(const imu::data& imu1, const imu::data& imu2,
                                            double last_stamp, Vec3_t& acc, Vec3_t& gyr, double& dt);
    static void preprocess_imu_interpolate2(const imu::data& imu1, const imu::data& imu2,
                                            double curr_stamp, Vec3_t& acc, Vec3_t& gyr, double& dt);

    //! Gather the keyframes by tracing the inertial reference keyframes
    static std::vector<openvslam::data::keyframe*> gather_intertial_ref_keyframes(openvslam::data::keyframe* keyfrm);

    //! Compute velocity
    static Vec3_t compute_velocity_interpolate(const openvslam::data::keyframe* keyfrm);
    static Vec3_t compute_velocity_extrapolate1(const openvslam::data::keyframe* keyfrm);
    static Vec3_t compute_velocity_extrapolate2(const openvslam::data::keyframe* keyfrm);
    static Vec3_t compute_velocity(const openvslam::data::keyframe* keyfrm);
    static void compute_velocity(std::vector<openvslam::data::keyframe*>& keyfrms);

    //! Compute gravity
    static Vec3_t compute_gravity(openvslam::data::keyframe* keyfrm);
    static Mat33_t compute_gravity_dir(std::vector<openvslam::data::keyframe*>& keyfrms);
};

} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_UTIL_H
