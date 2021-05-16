#ifndef OPENVSLAM_IMU_PREINTEGRATED_H
#define OPENVSLAM_IMU_PREINTEGRATED_H

#include <vector>
#include <memory>

#include "openvslam/type.h"
#include "openvslam/imu/bias.h"
#include <nlohmann/json_fwd.hpp>

// ref. Christian Forster, Luca Carlone, Frank Dellaert, Davide Scaramuzza, "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry" in IEEE Transactions on Robotics (TRO), 2016

namespace openvslam {
namespace imu {

class preintegrated {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Constructor for preintegrated IMU measurement
     * @param b
     */
    preintegrated(const bias& b);

    /**
     * Constructor for preintegrated IMU measurement
     * @param dt
     * @param covariance
     * @param b
     * @param delta_rotation
     * @param delta_velocity,
     * @param delta_position,
     * @param jacob_rotation_gyr,
     * @param jacob_velocity_gyr,
     * @param jacob_velocity_acc,
     * @param jacob_position_gyr,
     * @param jacob_position_acc
     */
    preintegrated(
        double dt,
        const MatRC_t<15, 15>& covariance,
        const bias& b,
        const Mat33_t& delta_rotation,
        const Vec3_t& delta_velocity,
        const Vec3_t& delta_position,
        const Mat33_t& jacob_rotation_gyr,
        const Mat33_t& jacob_velocity_gyr,
        const Mat33_t& jacob_velocity_acc,
        const Mat33_t& jacob_position_gyr,
        const Mat33_t& jacob_position_acc);

    //! Create preintegrated IMU measurement from json
    explicit preintegrated(const nlohmann::json& json_preintegrated);

    //! Inisialize states
    void initialize();

    //! Feed a new IMU measurement
    void integrate(const Vec3_t& acc, const Vec3_t& gyr, const double dt, const Mat66_t& initial_covariance, const Mat66_t& bias_covariance);

    //! Get delta rotation on bias b
    Mat33_t get_delta_rotation_on_bias(const imu::bias& b);

    //! Get delta velocity on bias b
    Vec3_t get_delta_velocity_on_bias(const imu::bias& b);

    //! Get delta position on bias b
    Vec3_t get_delta_position_on_bias(const imu::bias& b);

    //! Get information matrix
    MatRC_t<15, 15> get_information();

    //! Create json from preintegrated IMU measurement
    nlohmann::json to_json() const;

    //! Time elapsed since the first measurement
    double dt_;

    //! covariance
    MatRC_t<15, 15> covariance_;

    //! bias
    bias b_;

    //! delta rotaition
    Mat33_t delta_rotation_;

    //! delta velocity
    Vec3_t delta_velocity_;

    //! delta position
    Vec3_t delta_position_;

    //! jacobian of rotation with respect to gyro bias
    Mat33_t jacob_rotation_gyr_;

    //! jacobian of velocity with respect to gyro bias
    Mat33_t jacob_velocity_gyr_;

    //! jacobian of velocity with respect to acceleration bias
    Mat33_t jacob_velocity_acc_;

    //! jacobian of position with respect to gyro bias
    Mat33_t jacob_position_gyr_;

    //! jacobian of position with respect to acceleration bias
    Mat33_t jacob_position_acc_;
};

} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_PREINTEGRATED_H
