#include "openvslam/imu/preintegrator.h"
#include "openvslam/imu/preintegrated.h"
#include "openvslam/data/common.h"
#include <nlohmann/json.hpp>

namespace openvslam {
namespace imu {

measurement::measurement(const double acc_x, const double acc_y, const double acc_z,
                         const double gyr_x, const double gyr_y, const double gyr_z,
                         const double dt)
    : acc_(acc_x, acc_y, acc_z), gyr_(gyr_x, gyr_y, gyr_z), dt_(dt) {}

measurement::measurement(const Vec3_t& acc, const Vec3_t& gyr, const double dt)
    : acc_(acc), gyr_(gyr), dt_(dt) {}

preintegrator::preintegrator(const bias& b, const std::shared_ptr<config>& cfg) {
    initial_covariance_ << cfg->get_gyr_covariance(), Mat33_t::Zero(), Mat33_t::Zero(), cfg->get_acc_covariance();
    bias_covariance_ << cfg->get_gyr_bias_covariance(), Mat33_t::Zero(), Mat33_t::Zero(), cfg->get_acc_bias_covariance();
    preintegrated_ = eigen_alloc_shared<preintegrated>(b);
    preintegrated_->initialize();
}

preintegrator::preintegrator(const bias& b, const Mat66_t& initial_covariance, const Mat66_t& bias_covariance) {
    initial_covariance_ = initial_covariance;
    bias_covariance_ = bias_covariance;
    preintegrated_ = eigen_alloc_shared<preintegrated>(b);
    preintegrated_->initialize();
}

preintegrator::preintegrator(const nlohmann::json& json_preintegrator) {
    initial_covariance_ = data::convert_json_to_matrix<Mat66_t>(json_preintegrator.at("initial_covariance"));
    bias_covariance_ = data::convert_json_to_matrix<Mat66_t>(json_preintegrator.at("bias_covariance"));
    preintegrated_ = eigen_alloc_shared<preintegrated>(json_preintegrator.at("preintegrated"));
    for (const auto& json_mesurement : json_preintegrator.at("measurements")) {
        measurements_.emplace_back(
            data::convert_json_to_matrix<Vec3_t>(json_mesurement.at("acc")),
            data::convert_json_to_matrix<Vec3_t>(json_mesurement.at("gyr")),
            json_mesurement.at("dt").get<double>());
    }
}

void preintegrator::reintegrate(const imu::bias& b) {
    preintegrated_->b_ = b;
    preintegrated_->initialize();
    for (const auto& m : measurements_) {
        preintegrated_->integrate(m.acc_, m.gyr_, m.dt_, initial_covariance_, bias_covariance_);
    }
}

void preintegrator::merge_previous(const preintegrator& prev) {
    const auto tmp = measurements_;
    measurements_.clear();
    preintegrated_->initialize();
    for (const auto& m : prev.measurements_) {
        integrate_new_measurement(m);
    }
    for (const auto& m : tmp) {
        integrate_new_measurement(m);
    }
}

void preintegrator::integrate_new_measurement(const measurement& m) {
    measurements_.push_back(m);
    preintegrated_->integrate(m.acc_, m.gyr_, m.dt_, initial_covariance_, bias_covariance_);
}

void preintegrator::integrate_new_measurement(const Vec3_t& acc, const Vec3_t& gyr, const double dt) {
    measurements_.emplace_back(acc, gyr, dt);
    preintegrated_->integrate(acc, gyr, dt, initial_covariance_, bias_covariance_);
}

nlohmann::json preintegrator::to_json() const {
    nlohmann::json json_preintegrator;
    json_preintegrator["initial_covariance"] = data::convert_matrix_to_json(initial_covariance_);
    json_preintegrator["bias_covariance"] = data::convert_matrix_to_json(bias_covariance_);
    json_preintegrator["preintegrated"] = preintegrated_->to_json();
    for (const auto& m : measurements_) {
        nlohmann::json json_mesurement;
        json_mesurement["acc"] = data::convert_matrix_to_json(m.acc_);
        json_mesurement["gyr"] = data::convert_matrix_to_json(m.gyr_);
        json_mesurement["dt"] = m.dt_;
        json_preintegrator["measurements"].push_back(json_mesurement);
    }
    return json_preintegrator;
}
} // namespace imu
} // namespace openvslam
