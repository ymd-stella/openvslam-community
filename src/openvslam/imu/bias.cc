#include "openvslam/imu/bias.h"
#include "openvslam/data/common.h"
#include <nlohmann/json.hpp>

namespace openvslam {
namespace imu {

bias::bias(const float bax, const float bay, const float baz,
           const float bwx, const float bwy, const float bwz)
    : acc_((Vec3_t() << bax, bay, baz).finished()), gyr_((Vec3_t() << bwx, bwy, bwz).finished()) {}

bias::bias(const nlohmann::json& json_bias) {
    acc_ = data::convert_json_to_matrix<Vec3_t>(json_bias.at("acc"));
    gyr_ = data::convert_json_to_matrix<Vec3_t>(json_bias.at("gyr"));
}

nlohmann::json bias::to_json() const {
    return {{"acc", data::convert_matrix_to_json(acc_)},
            {"gyr", data::convert_matrix_to_json(gyr_)}};
}

} // namespace imu
} // namespace openvslam
