#ifndef OPENVSLAM_IMU_BIAS_H
#define OPENVSLAM_IMU_BIAS_H

#include "openvslam/type.h"

#include <nlohmann/json_fwd.hpp>

namespace openvslam {
namespace imu {

class bias {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Constructor for zero IMU bais
    bias()
        : bias(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) {}

    /**
     * Constructor for IMU bais
     * @param bax
     * @param bay
     * @param baz
     * @param bwx
     * @param bwy
     * @param bwz
     */
    bias(const float bax, const float bay, const float baz,
         const float bwx, const float bwy, const float bwz);

    /**
     * Constructor for IMU bais
     * @param acc
     * @param gyr
     */
    bias(const Vec3_t& acc, const Vec3_t& gyr)
        : acc_(acc), gyr_(gyr) {}

    /**
     * Create IMU bais from json
     * @param json_bias
     */
    explicit bias(const nlohmann::json& json_bias);

    /**
     * Create json from IMU bias
     * @param json_bias
     */
    nlohmann::json to_json() const;

    //! acceleration
    Vec3_t acc_;
    //! angular velocity (gyro)
    Vec3_t gyr_;
};

} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_BIAS_H
