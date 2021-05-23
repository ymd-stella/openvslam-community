#include "openvslam/imu/imu_util.h"
#include "openvslam/imu/data.h"

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

} // namespace imu
} // namespace openvslam
