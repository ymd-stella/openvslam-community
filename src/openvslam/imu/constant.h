#ifndef OPENVSLAM_IMU_CONSTANT_H
#define OPENVSLAM_IMU_CONSTANT_H

namespace openvslam {
namespace imu {

class constant {
public:
    //! Gravity of Earth
    static constexpr double gravity() {
        return 9.81;
    }
};

} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_CONSTANT_H
