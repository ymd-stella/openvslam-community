#include "openvslam/imu/config.h"
#include "openvslam/data/common.h"
#include <nlohmann/json.hpp>

namespace openvslam {
namespace imu {

config::config(const std::string& name, const double rate_hz, const Mat44_t& rel_pose_ic,
               const double ns_acc, const double ns_gyr, const double rw_acc_bias, const double rw_gyr_bias)
    : name_(name), rate_hz_(rate_hz), rate_dt_(1.0 / rate_hz), rel_pose_ic_(rel_pose_ic),
      ns_acc_(ns_acc), ns_gyr_(ns_gyr), rw_acc_bias_(rw_acc_bias), rw_gyr_bias_(rw_gyr_bias) {
    update_pose();
    update_covariance();
}

config::config(const YAML::Node& yaml_node)
    : config(yaml_node["name"].as<std::string>(),
             yaml_node["rate_hz"].as<unsigned int>(),
             Mat44_t(yaml_node["rel_pose_ic"].as<std::vector<double>>().data()).transpose(),
             yaml_node["ns_acc"].as<double>(),
             yaml_node["ns_gyr"].as<double>(),
             yaml_node["rw_acc_bias"].as<double>(),
             yaml_node["rw_gyr_bias"].as<double>()) {}

config::config(const nlohmann::json& json_imu_config)
    : config(json_imu_config.at("name").get<std::string>(),
             json_imu_config.at("rate_hz").get<double>(),
             data::convert_json_to_matrix<Mat44_t>(json_imu_config.at("rel_pose_ic")),
             json_imu_config.at("ns_acc").get<double>(),
             json_imu_config.at("ns_gyr").get<double>(),
             json_imu_config.at("rw_acc_bias").get<double>(),
             json_imu_config.at("rw_gyr_bias").get<double>()) {
    update_pose();
    update_covariance();
}

nlohmann::json config::to_json() const {
    nlohmann::json json_imu_config;
    json_imu_config["name"] = name_;
    json_imu_config["rate_hz"] = rate_hz_;
    json_imu_config["rel_pose_ic"] = data::convert_matrix_to_json(rel_pose_ic_);
    json_imu_config["ns_acc"] = ns_acc_;
    json_imu_config["ns_gyr"] = ns_gyr_;
    json_imu_config["rw_acc_bias"] = rw_acc_bias_;
    json_imu_config["rw_gyr_bias"] = rw_gyr_bias_;
    return json_imu_config;
}

std::string config::get_name() const {
    return name_;
}

double config::get_rate_hz() const {
    return rate_hz_;
}

double config::get_rate_dt() const {
    return rate_dt_;
}

Mat44_t config::get_rel_pose_ic() const {
    return rel_pose_ic_;
}

Mat33_t config::get_rel_rot_ic() const {
    return rel_pose_ic_.block<3, 3>(0, 0);
}

Vec3_t config::get_rel_trans_ic() const {
    return rel_pose_ic_.block<3, 1>(0, 3);
}

Mat44_t config::get_rel_pose_ci() const {
    return rel_pose_ci_;
}

Mat33_t config::get_rel_rot_ci() const {
    return rel_pose_ci_.block<3, 3>(0, 0);
}

Vec3_t config::get_rel_trans_ci() const {
    return rel_pose_ci_.block<3, 1>(0, 3);
}

void config::set_acc_noise_density(const double ns_acc) {
    ns_acc_ = ns_acc;
    update_covariance();
}

void config::set_gyr_noise_density(const double ns_gyr) {
    ns_gyr_ = ns_gyr;
    update_covariance();
}

void config::set_acc_bias_random_walk(const double rw_acc_bias) {
    rw_acc_bias_ = rw_acc_bias;
    update_covariance();
}

void config::set_gyr_bias_random_walk(const double rw_gyr_bias) {
    rw_gyr_bias_ = rw_gyr_bias;
    update_covariance();
}

Mat33_t config::get_acc_covariance() const {
    return cov_acc_;
}

Mat33_t config::get_gyr_covariance() const {
    return cov_gyr_;
}

Mat33_t config::get_acc_bias_covariance() const {
    return cov_acc_bias_;
}

Mat33_t config::get_gyr_bias_covariance() const {
    return cov_gyr_bias_;
}

void config::update_pose() {
    const Mat33_t rel_rot_ic = rel_pose_ic_.block<3, 3>(0, 0);
    const Vec3_t rel_trans_ic = rel_pose_ic_.block<3, 1>(0, 3);
    rel_pose_ci_ = Mat44_t::Identity();
    rel_pose_ci_.block<3, 3>(0, 0) = rel_rot_ic.transpose();
    rel_pose_ci_.block<3, 1>(0, 3) = -rel_rot_ic.transpose() * rel_trans_ic;
}

void config::update_covariance() {
    cov_acc_ = Mat33_t::Identity() * ns_acc_ * ns_acc_ * rate_hz_;
    cov_gyr_ = Mat33_t::Identity() * ns_gyr_ * ns_gyr_ * rate_hz_;
    cov_acc_bias_ = Mat33_t::Identity() * rw_acc_bias_ * rw_acc_bias_ * rate_hz_;
    cov_gyr_bias_ = Mat33_t::Identity() * rw_gyr_bias_ * rw_gyr_bias_ * rate_hz_;
}

} // namespace imu
} // namespace openvslam
