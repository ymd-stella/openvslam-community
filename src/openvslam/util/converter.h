#ifndef OPENVSLAM_UTIL_CONVERTER_H
#define OPENVSLAM_UTIL_CONVERTER_H

#include "openvslam/type.h"

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

namespace openvslam {
namespace util {

class converter {
public:
    //! descriptor vector
    static std::vector<cv::Mat> to_desc_vec(const cv::Mat& desc);

    //! to SE3 of g2o
    static g2o::SE3Quat to_g2o_SE3(const Mat44_t& cam_pose);

    //! to Eigen::Mat/Vec
    static Mat44_t to_eigen_mat(const g2o::SE3Quat& g2o_SE3);
    static Mat44_t to_eigen_mat(const g2o::Sim3& g2o_Sim3);
    static Mat44_t to_eigen_cam_pose(const Mat33_t& rot, const Vec3_t& trans);

    //! from/to angle axis
    static Vec3_t to_angle_axis(const Mat33_t& rot_mat);
    static Mat33_t to_rot_mat(const Vec3_t& angle_axis);

    //! to homogeneous coordinates
    template<typename T>
    static Vec3_t to_homogeneous(const cv::Point_<T>& pt) {
        return Vec3_t{pt.x, pt.y, 1.0};
    }

    //! to skew symmetric matrix
    static Mat33_t to_skew_symmetric_mat(const Vec3_t& vec);

    //! normalize rotation
    static Mat33_t normalize_rotation(const Mat33_t& R);

    //! SO(3) matrix exponential. Mapping from the vector to SO(3) lie group
    static Mat33_t exp_so3(const Vec3_t& v);

    //! Vector portion of skew-symmetric
    static Vec3_t vee(const Mat33_t& R);

    //! SO(3) matrix logarithm
    static Vec3_t log_so3(const Mat33_t& R);

    //! Inverse matrix of right jacobian
    static Mat33_t inverse_right_jacobian_so3(const Vec3_t& v);

    //! Right jacobian. The reference is "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", (8)
    static Mat33_t right_jacobian_so3(const Vec3_t& v);
};

} // namespace util
} // namespace openvslam

#endif // OPENVSLAM_UTIL_CONVERTER_H
