#include "openvslam/type.h"
#include "openvslam/util/converter.h"

#include <gtest/gtest.h>
#include <iostream>

using namespace openvslam;

TEST(converter, normalize_rotation) {
    Mat33_t rot;
    rot << 2, 0, 0, 0, 2, 0, 0, 0, 2;
    const Mat33_t ret1 = util::converter::normalize_rotation(rot);
    EXPECT_TRUE(ret1.isApprox(rot / 2.0));
    rot = Eigen::AngleAxisd(1.0, Vec3_t(1.0, 0.0, 0.0))
          * Eigen::AngleAxisd(1.0, Vec3_t(0.0, 1.0, 0.0))
          * Eigen::AngleAxisd(1.0, Vec3_t(0.0, 0.0, 1.0)) * rot;
    const Mat33_t ret2 = util::converter::normalize_rotation(rot);
    EXPECT_TRUE(Mat33_t::Identity().isApprox(ret2 * ret2.transpose()));
}

TEST(converter, exp_so3) {
    const Mat33_t rot = util::converter::exp_so3(Vec3_t(1.0, 1.0, 1.0));
    EXPECT_TRUE(Mat33_t::Identity().isApprox(rot * rot.transpose()));
    EXPECT_TRUE(Mat33_t::Identity().isApprox(util::converter::exp_so3(Vec3_t(0.0, 0.0, 0.0))));
}

TEST(converter, right_jacobian_so3) {
    const Vec3_t delta = Vec3_t(0.1, 0.1, 0.1);
    const Vec3_t v = Vec3_t(1.0, 1.0, 1.0);
    const Mat33_t rot = util::converter::exp_so3(v);
    const Mat33_t right_jacobian = util::converter::right_jacobian_so3(v);
    const Mat33_t ref = util::converter::exp_so3(v + delta);
    EXPECT_TRUE(ref.isApprox(rot * util::converter::exp_so3(right_jacobian * delta)));
    EXPECT_TRUE(Mat33_t::Identity().isApprox(util::converter::right_jacobian_so3(Vec3_t(0.0, 0.0, 0.0))));
}
