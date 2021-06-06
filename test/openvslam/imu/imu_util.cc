#include "openvslam/imu/imu_util.h"
#include "openvslam/imu/data.h"
#include "openvslam/imu/preintegrated.h"

#include <gtest/gtest.h>

using namespace openvslam;

TEST(data, preprocess_imu) {
    imu::data d1(
        Vec3_t(1.0, 1.0, 1.0),
        Vec3_t(1.0, 1.0, 1.0),
        0.0);
    imu::data d2(
        Vec3_t(2.0, 2.0, 2.0),
        Vec3_t(2.0, 2.0, 2.0),
        1.0);
    Vec3_t acc2, gyr2;
    double dt2;
    imu::imu_util::preprocess_imu(d1, d2, acc2, gyr2, dt2);

    Vec3_t acc(1.5, 1.5, 1.5);
    Vec3_t gyr(1.5, 1.5, 1.5);
    double dt = 1.0;
    EXPECT_TRUE(acc2.isApprox(acc));
    EXPECT_TRUE(gyr2.isApprox(gyr));
    EXPECT_NEAR(dt2, dt, 1e-6);
}

TEST(data, preprocess_imu_interpolate1) {
    imu::data d1(
        Vec3_t(1.0, 1.0, 1.0),
        Vec3_t(1.0, 1.0, 1.0),
        0.0);
    imu::data d2(
        Vec3_t(2.0, 2.0, 2.0),
        Vec3_t(2.0, 2.0, 2.0),
        1.0);
    double t_prev = 0.5;
    Vec3_t acc2, gyr2;
    double dt2;
    imu::imu_util::preprocess_imu_interpolate1(d1, d2, t_prev, acc2, gyr2, dt2);

    Vec3_t acc(1.75, 1.75, 1.75);
    Vec3_t gyr(1.75, 1.75, 1.75);
    double dt = 0.5;
    EXPECT_TRUE(acc2.isApprox(acc));
    EXPECT_TRUE(gyr2.isApprox(gyr));
    EXPECT_NEAR(dt2, dt, 1e-6);
}

TEST(data, preprocess_imu_interpolate2) {
    imu::data d1(
        Vec3_t(1.0, 1.0, 1.0),
        Vec3_t(1.0, 1.0, 1.0),
        0.0);
    imu::data d2(
        Vec3_t(2.0, 2.0, 2.0),
        Vec3_t(2.0, 2.0, 2.0),
        1.0);
    double t_curr = 0.5;
    Vec3_t acc2, gyr2;
    double dt2;
    imu::imu_util::preprocess_imu_interpolate2(d1, d2, t_curr, acc2, gyr2, dt2);

    Vec3_t acc(1.25, 1.25, 1.25);
    Vec3_t gyr(1.25, 1.25, 1.25);
    double dt = 0.5;
    EXPECT_TRUE(acc2.isApprox(acc));
    EXPECT_TRUE(gyr2.isApprox(gyr));
    EXPECT_NEAR(dt2, dt, 1e-6);
}
