#include <openvslam/imu/internal/inertial_gravity_scale_edge_on_camera.h>
#include <openvslam/imu/internal/inertial_gravity_scale_edge_on_imu.h>
#include <openvslam/imu/internal/prior_bias_edge.h>

#include <gtest/gtest.h>

#include <sstream>
#include <iomanip>
#include <limits>

using namespace openvslam;

TEST(imu_edges, io_inertial_gravity_scale_edge_on_imu) {
    double dT = 0.25;
    MatRC_t<15, 15> C;
    C << 7.225e-09, -1.27462e-16, 6.0359e-17, -7.31754e-11, 4.33882e-09, -1.67116e-10, -6.12624e-12, 3.48724e-10, -9.93679e-12, 0, 0, 0, 0, 0, 0, -1.27394e-16, 7.225e-09, -5.36869e-16, -3.69204e-09, -2.7057e-11, -9.38122e-09, -2.96494e-10, -2.15074e-12, -7.5784e-10, 0, 0, 0, 0, 0, 0, 5.89788e-17, -5.36918e-16, 7.225e-09, 7.40036e-12, 9.10212e-09, 5.82739e-11, -2.97597e-12, 7.35347e-10, 4.45787e-12, 0, 0, 0, 0, 0, 0, -7.31754e-11, -3.69204e-09, 7.40037e-12, 1.00251e-06, -4.61832e-11, 6.38393e-09, 1.25226e-07, -4.83166e-12, 5.78846e-10, 0, 0, 0, 0, 0, 0, 4.33882e-09, -2.70571e-11, 9.10212e-09, -4.61832e-11, 1.01875e-06, 1.81194e-11, -9.6421e-12, 1.267e-07, 3.78773e-12, 0, 0, 0, 0, 0, 0, -1.67116e-10, -9.38121e-09, 5.82739e-11, 6.38393e-09, 1.81194e-11, 1.01624e-06, 5.76379e-10, 1.88945e-12, 1.26474e-07, 0, 0, 0, 0, 0, 0, -6.12624e-12, -2.96494e-10, -2.97597e-12, 1.25226e-07, -9.64209e-12, 5.76379e-10, 2.08531e-08, -9.14081e-13, 5.58616e-11, 0, 0, 0, 0, 0, 0, 3.48724e-10, -2.15074e-12, 7.35346e-10, -4.83166e-12, 1.267e-07, 1.88946e-12, -9.14081e-13, 2.09959e-08, 3.57514e-13, 0, 0, 0, 0, 0, 0, -9.93678e-12, -7.5784e-10, 4.45787e-12, 5.78846e-10, 3.78774e-12, 1.26474e-07, 5.58616e-11, 3.57514e-13, 2.09741e-08, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3.76089e-06, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3.76089e-06, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3.76089e-06, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.09, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.09, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.09;
    Mat33_t dR;
    dR << 0.997363, -0.0199074, -0.0697836, 0.0195129, 0.99979, -0.00632964, 0.0698949, 0.00495126, 0.997542;
    Vec3_t dV;
    dV << 2.60269, 0.0085997, -1.02248;
    Vec3_t dP;
    dP << 0.319498, 0.00157434, -0.125288;
    Mat33_t JRg;
    JRg << -0.249855, -0.000426387, -0.00647477, 0.000439231, -0.249993, -0.000361433, 0.0064727, 0.000405747, -0.249861;
    Mat33_t JVg;
    JVg << 0.00188871, 0.12776, 4.47947e-05, -0.138147, 0.000815634, -0.320308, 0.00423786, 0.324622, -0.00129467;
    Mat33_t JVa;
    JVa << -0.249664, 0.00447337, 0.0108113, -0.00442846, -0.249952, 0.00107803, -0.0108299, -0.000850437, -0.249708;
    Mat33_t JPg;
    JPg << 0.000129729, 0.0102601, 0.000131011, -0.0109212, 4.29987e-05, -0.0259519, 0.000152358, 0.0262243, -9.10077e-05;
    Mat33_t JPa;
    JPa << -0.0312242, 0.000453093, 0.000978035, -0.000448955, -0.0312455, 0.000121724, -0.00097996, -0.000102707, -0.0312282;
    Vec3_t acc;
    acc << 0.1, 0.2, 0.3;
    Vec3_t gyr;
    acc << 0.4, 0.5, 0.6;
    std::shared_ptr<imu::preintegrated> preintegrated = eigen_alloc_shared<imu::preintegrated>(dT, C, imu::bias(acc, gyr), dR, dV, dP, JRg, JVg, JVa, JPg, JPa);

    imu::internal::inertial_gravity_scale_edge_on_imu edge;
    edge.setInformation(preintegrated->get_information().block<9, 9>(0, 0));
    edge.setMeasurement(preintegrated);

    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<double>::max_digits10) << std::scientific;
    edge.write(ss);

    imu::internal::inertial_gravity_scale_edge_on_imu edge2;
    edge2.read(ss);

    EXPECT_TRUE(edge.information().isApprox(edge2.information()));
    EXPECT_LT(std::abs(edge2.measurement()->dt_ - dT), 1e-6);
    EXPECT_TRUE(edge2.measurement()->covariance_.isApprox(C));
    EXPECT_TRUE(edge2.measurement()->b_.acc_.isApprox(acc));
    EXPECT_TRUE(edge2.measurement()->b_.gyr_.isApprox(gyr));
    EXPECT_TRUE(edge2.measurement()->jacob_rotation_gyr_.isApprox(JRg));
    EXPECT_TRUE(edge2.measurement()->jacob_velocity_gyr_.isApprox(JVg));
    EXPECT_TRUE(edge2.measurement()->jacob_velocity_acc_.isApprox(JVa));
    EXPECT_TRUE(edge2.measurement()->jacob_position_gyr_.isApprox(JPg));
    EXPECT_TRUE(edge2.measurement()->jacob_position_acc_.isApprox(JPa));
    EXPECT_TRUE(edge2.measurement()->delta_rotation_.isApprox(dR));
    EXPECT_TRUE(edge2.measurement()->delta_velocity_.isApprox(dV));
    EXPECT_TRUE(edge2.measurement()->delta_position_.isApprox(dP));
}

TEST(imu_edges, io_inertial_gravity_scale_edge_on_camera) {
    double dT = 0.25;
    MatRC_t<15, 15> C;
    C << 7.225e-09, -1.27462e-16, 6.0359e-17, -7.31754e-11, 4.33882e-09, -1.67116e-10, -6.12624e-12, 3.48724e-10, -9.93679e-12, 0, 0, 0, 0, 0, 0, -1.27394e-16, 7.225e-09, -5.36869e-16, -3.69204e-09, -2.7057e-11, -9.38122e-09, -2.96494e-10, -2.15074e-12, -7.5784e-10, 0, 0, 0, 0, 0, 0, 5.89788e-17, -5.36918e-16, 7.225e-09, 7.40036e-12, 9.10212e-09, 5.82739e-11, -2.97597e-12, 7.35347e-10, 4.45787e-12, 0, 0, 0, 0, 0, 0, -7.31754e-11, -3.69204e-09, 7.40037e-12, 1.00251e-06, -4.61832e-11, 6.38393e-09, 1.25226e-07, -4.83166e-12, 5.78846e-10, 0, 0, 0, 0, 0, 0, 4.33882e-09, -2.70571e-11, 9.10212e-09, -4.61832e-11, 1.01875e-06, 1.81194e-11, -9.6421e-12, 1.267e-07, 3.78773e-12, 0, 0, 0, 0, 0, 0, -1.67116e-10, -9.38121e-09, 5.82739e-11, 6.38393e-09, 1.81194e-11, 1.01624e-06, 5.76379e-10, 1.88945e-12, 1.26474e-07, 0, 0, 0, 0, 0, 0, -6.12624e-12, -2.96494e-10, -2.97597e-12, 1.25226e-07, -9.64209e-12, 5.76379e-10, 2.08531e-08, -9.14081e-13, 5.58616e-11, 0, 0, 0, 0, 0, 0, 3.48724e-10, -2.15074e-12, 7.35346e-10, -4.83166e-12, 1.267e-07, 1.88946e-12, -9.14081e-13, 2.09959e-08, 3.57514e-13, 0, 0, 0, 0, 0, 0, -9.93678e-12, -7.5784e-10, 4.45787e-12, 5.78846e-10, 3.78774e-12, 1.26474e-07, 5.58616e-11, 3.57514e-13, 2.09741e-08, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3.76089e-06, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3.76089e-06, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3.76089e-06, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.09, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.09, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.09;
    Mat33_t dR;
    dR << 0.997363, -0.0199074, -0.0697836, 0.0195129, 0.99979, -0.00632964, 0.0698949, 0.00495126, 0.997542;
    Vec3_t dV;
    dV << 2.60269, 0.0085997, -1.02248;
    Vec3_t dP;
    dP << 0.319498, 0.00157434, -0.125288;
    Mat33_t JRg;
    JRg << -0.249855, -0.000426387, -0.00647477, 0.000439231, -0.249993, -0.000361433, 0.0064727, 0.000405747, -0.249861;
    Mat33_t JVg;
    JVg << 0.00188871, 0.12776, 4.47947e-05, -0.138147, 0.000815634, -0.320308, 0.00423786, 0.324622, -0.00129467;
    Mat33_t JVa;
    JVa << -0.249664, 0.00447337, 0.0108113, -0.00442846, -0.249952, 0.00107803, -0.0108299, -0.000850437, -0.249708;
    Mat33_t JPg;
    JPg << 0.000129729, 0.0102601, 0.000131011, -0.0109212, 4.29987e-05, -0.0259519, 0.000152358, 0.0262243, -9.10077e-05;
    Mat33_t JPa;
    JPa << -0.0312242, 0.000453093, 0.000978035, -0.000448955, -0.0312455, 0.000121724, -0.00097996, -0.000102707, -0.0312282;
    Vec3_t acc;
    acc << 0.1, 0.2, 0.3;
    Vec3_t gyr;
    acc << 0.4, 0.5, 0.6;
    std::shared_ptr<imu::preintegrated> preintegrated = eigen_alloc_shared<imu::preintegrated>(dT, C, imu::bias(acc, gyr), dR, dV, dP, JRg, JVg, JVa, JPg, JPa);

    imu::internal::inertial_gravity_scale_edge_on_camera edge(Mat33_t::Identity(), Vec3_t::Zero());
    edge.setInformation(preintegrated->get_information().block<9, 9>(0, 0));
    edge.setMeasurement(preintegrated);

    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<double>::max_digits10) << std::scientific;
    edge.write(ss);

    imu::internal::inertial_gravity_scale_edge_on_camera edge2(Mat33_t::Identity(), Vec3_t::Zero());
    edge2.read(ss);

    EXPECT_TRUE(edge.information().isApprox(edge2.information()));
    EXPECT_LT(std::abs(edge2.measurement()->dt_ - dT), 1e-6);
    EXPECT_TRUE(edge2.measurement()->covariance_.isApprox(C));
    EXPECT_TRUE(edge2.measurement()->b_.acc_.isApprox(acc));
    EXPECT_TRUE(edge2.measurement()->b_.gyr_.isApprox(gyr));
    EXPECT_TRUE(edge2.measurement()->jacob_rotation_gyr_.isApprox(JRg));
    EXPECT_TRUE(edge2.measurement()->jacob_velocity_gyr_.isApprox(JVg));
    EXPECT_TRUE(edge2.measurement()->jacob_velocity_acc_.isApprox(JVa));
    EXPECT_TRUE(edge2.measurement()->jacob_position_gyr_.isApprox(JPg));
    EXPECT_TRUE(edge2.measurement()->jacob_position_acc_.isApprox(JPa));
    EXPECT_TRUE(edge2.measurement()->delta_rotation_.isApprox(dR));
    EXPECT_TRUE(edge2.measurement()->delta_velocity_.isApprox(dV));
    EXPECT_TRUE(edge2.measurement()->delta_position_.isApprox(dP));
}

TEST(imu_edges, io_prior_bias_edge) {
    imu::internal::prior_bias_edge edge;
    edge.setInformation(Mat33_t::Identity());
    edge.setMeasurement(Vec3_t(0.1, 0.2, 0.3));

    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<double>::max_digits10) << std::scientific;
    edge.write(ss);

    imu::internal::prior_bias_edge edge2;
    edge2.read(ss);

    EXPECT_TRUE(edge.information().isApprox(edge2.information()));
    EXPECT_TRUE(edge.measurement().isApprox(edge2.measurement()));
}
