#ifndef OPENVSLAM_IMU_IMU_DATABASE_H
#define OPENVSLAM_IMU_IMU_DATABASE_H

#include <mutex>
#include <unordered_map>

#include <nlohmann/json_fwd.hpp>

namespace openvslam {

namespace imu {

class imu_database {
public:
    explicit imu_database(const std::shared_ptr<config>& curr_imu);

    ~imu_database();

    std::shared_ptr<config> get_imu(const std::string& imu_name) const;

    void from_json(const nlohmann::json& json_cameras);

    nlohmann::json to_json() const;

private:
    //-----------------------------------------
    //! mutex to access the database
    mutable std::mutex mtx_database_;
    //! pointer to the imu which used in the current tracking
    std::shared_ptr<config> curr_imu_ = nullptr;
    //! database (key: imu name, value: pointer of config)
    //! (NOTE: tracking imu must NOT be contained in the database)
    std::unordered_map<std::string, std::shared_ptr<config>> database_;
};

} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_IMU_DATABASE_H
