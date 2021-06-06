#ifndef OPENVSLAM_IMU_UTIL_H
#define OPENVSLAM_IMU_UTIL_H

#include <vector>

#include "openvslam/type.h"

namespace openvslam {
namespace imu {
class data;

class imu_util {
public:
    static void preprocess_imu(const imu::data& imu1, const imu::data& imu2,
                               Vec3_t& acc, Vec3_t& gyr, double& dt);
    static void preprocess_imu_interpolate1(const imu::data& imu1, const imu::data& imu2,
                                            double last_stamp, Vec3_t& acc, Vec3_t& gyr, double& dt);
    static void preprocess_imu_interpolate2(const imu::data& imu1, const imu::data& imu2,
                                            double curr_stamp, Vec3_t& acc, Vec3_t& gyr, double& dt);
};

} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_UTIL_H
