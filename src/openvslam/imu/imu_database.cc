#include "openvslam/imu/config.h"
#include "openvslam/imu/imu_database.h"

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace openvslam {
namespace imu {

imu_database::imu_database(const std::shared_ptr<config>& curr_imu)
    : curr_imu_(curr_imu) {
    spdlog::debug("CONSTRUCT: imu::imu_database");
    if (curr_imu_) {
        database_[curr_imu->get_name()] = curr_imu;
    }
}

imu_database::~imu_database() {
    spdlog::debug("DESTRUCT: imu::imu_database");
}

std::shared_ptr<config> imu_database::get_imu(const std::string& imu_name) const {
    std::lock_guard<std::mutex> lock(mtx_database_);
    if (imu_name == curr_imu_->get_name()) {
        return curr_imu_;
    }
    else {
        assert(database_.count(imu_name));
        return database_.at(imu_name);
    }
}

void imu_database::from_json(const nlohmann::json& json_imus) {
    std::lock_guard<std::mutex> lock(mtx_database_);

    for (const auto& json_id_camera : json_imus.items()) {
        const auto& imu_name = json_id_camera.key();
        const auto& json_imu = json_id_camera.value();

        if (imu_name == curr_imu_->get_name()) {
            spdlog::info("skip loading the tracking imu \"{}\" from JSON", imu_name);
            continue;
        }

        spdlog::info("load a imu \"{}\" from JSON", imu_name);
        assert(!database_.count(imu_name));
        database_[imu_name] = eigen_alloc_shared<config>(json_imu);
    }
}

nlohmann::json imu_database::to_json() const {
    std::lock_guard<std::mutex> lock(mtx_database_);

    nlohmann::json json_imus;
    for (const auto& name_imu : database_) {
        const auto& imu_name = name_imu.first;
        const auto imu = name_imu.second;
        json_imus[imu_name] = imu->to_json();
    }
    return json_imus;
}

} // namespace imu
} // namespace openvslam
