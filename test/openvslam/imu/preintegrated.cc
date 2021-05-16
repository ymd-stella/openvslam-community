#include "openvslam/imu/preintegrator.h"
#include "openvslam/imu/preintegrated.h"
#include "openvslam/imu/config.h"
#include "openvslam/imu/constant.h"
#include "openvslam/imu/data.h"
#include <iostream>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

using namespace openvslam;

TEST(data, reintegrate_zero) {
    // basic information
    const std::string name = "IMU";

    // create the relative pose "from IMU to camera" (_ic) and "from camera to IMU" (_ci)
    Mat44_t rel_pose_ic;
    rel_pose_ic << 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;

    // Fake values, taken from EuRoC parameters.
    const double rate_hz = 200.0;
    // acc noise [(m/s^2)/sqrt(Hz)]
    const double ns_acc = 2.0000e-3;
    // gyr noise [(rad/s]/sqrt(Hz)]
    const double ns_gyr = 1.7e-4;
    // acc bias random walk [(m/s^3)/sqrt(Hz)]
    const double rw_acc_bias = 3.0000e-03;
    // gyr bias random walk [(rad/s^2]/sqrt(Hz)]
    const double rw_gyr_bias = 1.9393e-05;

    const auto cfg = eigen_alloc_shared<imu::config>(name, rate_hz, rel_pose_ic, ns_acc, ns_gyr, rw_acc_bias, rw_gyr_bias);

    imu::preintegrator p(imu::bias(), cfg);

    p.measurements_.emplace_back(Vec3_t(0., 0., 0.), Vec3_t(0., 0., 0.), 0.1);

    p.reintegrate(imu::bias());
    EXPECT_NEAR(p.preintegrated_->dt_, 0.1, 1e-6);
    EXPECT_TRUE(Mat33_t::Identity().isApprox(p.preintegrated_->delta_rotation_));
    EXPECT_EQ((p.preintegrated_->delta_velocity_).cwiseAbs().sum(), 0);
    EXPECT_EQ((p.preintegrated_->delta_position_).cwiseAbs().sum(), 0);
}

std::shared_ptr<imu::config> get_imu_config() {
    // basic information
    const std::string name = "IMU";

    // create the relative pose "from IMU to camera" (_ic) and "from camera to IMU" (_ci)
    Mat44_t rel_pose_ic;
    rel_pose_ic << 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;

    // Fake values, taken from EuRoC parameters.
    const double rate_hz = 200.0;
    // acc noise [(m/s^2)/sqrt(Hz)]
    const double ns_acc = 2.0000e-3;
    // gyr noise [(rad/s]/sqrt(Hz)]
    const double ns_gyr = 1.7e-4;
    // acc bias random walk [(m/s^3)/sqrt(Hz)]
    const double rw_acc_bias = 3.0000e-03;
    // gyr bias random walk [(rad/s^2]/sqrt(Hz)]
    const double rw_gyr_bias = 1.9393e-05;

    return eigen_alloc_shared<imu::config>(name, rate_hz, rel_pose_ic, ns_acc, ns_gyr, rw_acc_bias, rw_gyr_bias);
}

void show_preintegrated(const imu::preintegrated& preintegrated) {
    const Vec3_t G(0.0, 0.0, -imu::constant::gravity());

    const auto show = std::getenv("SHOW_IMU_PREINTEGRATED");
    const std::string show_str = (show != nullptr) ? show : "";
    if (show_str == "YES" || show_str == "ON" || show_str == "TRUE" || show_str == "1") {
        std::cout << "jacob_rotation_gyr: " << preintegrated.jacob_rotation_gyr_ << std::endl;
        std::cout << "jacob_velocity_gyr: " << preintegrated.jacob_velocity_gyr_ << std::endl;
        std::cout << "jacob_velocity_acc: " << preintegrated.jacob_velocity_acc_ << std::endl;
        std::cout << "jacob_position_gyr: " << preintegrated.jacob_position_gyr_ << std::endl;
        std::cout << "jacob_position_acc: " << preintegrated.jacob_position_acc_ << std::endl;
        std::cout << "dt: " << preintegrated.dt_ << std::endl;
        std::cout << "delta_rotation: " << preintegrated.delta_rotation_ << std::endl;
        std::cout << "delta_rotation_angle: " << Eigen::AngleAxisd(preintegrated.delta_rotation_).angle() << std::endl;
        std::cout << "delta_velocity: " << preintegrated.delta_velocity_ << std::endl;
        std::cout << "vel: " << G * preintegrated.dt_ + preintegrated.delta_velocity_ << std::endl;
        std::cout << "delta_position: " << preintegrated.delta_position_ << std::endl;
        std::cout << "pos: " << 0.5 * G * preintegrated.dt_ * preintegrated.dt_ + preintegrated.delta_position_ << std::endl;
    }
}

TEST(data, reintegrate_linear) {
    const Vec3_t G(0.0, 0.0, -imu::constant::gravity());
    auto cfg = get_imu_config();

    std::vector<imu::data> imu_data;
    double timestamp = 0.0;
    for (unsigned int i = 0; i < 11; ++i) {
        Vec3_t gyr(0.0, 0.0, 0.0);
        Vec3_t acc(1.0, 0.0, imu::constant::gravity());
        imu_data.emplace_back(acc, gyr, timestamp);
        timestamp += 0.01;
    }

    imu::preintegrator p(imu::bias(), cfg);
    for (unsigned int i = 0; i < imu_data.size() - 1; ++i) {
        auto d1 = imu_data[i];
        auto d2 = imu_data[i + 1];
        Vec3_t acc1 = d1.acc_;
        Vec3_t gyr1 = d1.gyr_;
        Vec3_t acc2 = d2.acc_;
        Vec3_t gyr2 = d2.gyr_;
        double dt = d2.ts_ - d1.ts_;
        Vec3_t acc = (acc1 + acc2) * 0.5;
        Vec3_t gyr = (gyr1 + gyr2) * 0.5;
        p.measurements_.emplace_back(acc, gyr, dt);
    }

    p.reintegrate(imu::bias());
    auto preintegrated = *p.preintegrated_;

    EXPECT_NEAR(p.preintegrated_->dt_, 0.1, 1e-6);
    EXPECT_TRUE(Mat33_t::Identity().isApprox(p.preintegrated_->delta_rotation_));
    EXPECT_TRUE(Vec3_t(0.1, 0.0, 0.0).isApprox(G * preintegrated.dt_ + preintegrated.delta_velocity_));
    EXPECT_TRUE(Vec3_t(0.005, 0.0, 0.0).isApprox(0.5 * G * preintegrated.dt_ * preintegrated.dt_ + preintegrated.delta_position_));
    show_preintegrated(preintegrated);
}

TEST(data, reintegrate_angular) {
    const Vec3_t G(0.0, 0.0, -imu::constant::gravity());
    auto cfg = get_imu_config();

    std::vector<imu::data> imu_data;
    double timestamp = 0.0;
    for (unsigned int i = 0; i < 11; ++i) {
        Vec3_t gyr(0.1, 0.0, 0.0);
        Vec3_t acc(0.0, 0.0, imu::constant::gravity());
        imu_data.emplace_back(acc, gyr, timestamp);
        timestamp += 0.01;
    }

    imu::preintegrator p(imu::bias(), cfg);
    for (unsigned int i = 0; i < imu_data.size() - 1; ++i) {
        auto d1 = imu_data[i];
        auto d2 = imu_data[i + 1];
        Vec3_t acc1 = d1.acc_;
        Vec3_t gyr1 = d1.gyr_;
        Vec3_t acc2 = d2.acc_;
        Vec3_t gyr2 = d2.gyr_;
        double dt = d2.ts_ - d1.ts_;
        Vec3_t acc = (acc1 + acc2) * 0.5;
        Vec3_t gyr = (gyr1 + gyr2) * 0.5;
        p.measurements_.emplace_back(acc, gyr, dt);
    }

    p.reintegrate(imu::bias());
    auto preintegrated = *p.preintegrated_;

    EXPECT_NEAR(p.preintegrated_->dt_, 0.1, 1e-6);
    EXPECT_NEAR(Eigen::AngleAxisd(preintegrated.delta_rotation_).angle(), 0.01, 1e-6);
    show_preintegrated(preintegrated);
}
