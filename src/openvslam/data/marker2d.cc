#include "openvslam/data/marker2d.h"

namespace openvslam {
namespace data {

marker2d::marker2d(const std::vector<cv::Point2f>& undist_corners, const eigen_alloc_vector<Vec3_t>& bearings,
                   const Mat33_t& rot_cm, const Vec3_t& trans_cm, unsigned int id)
    : undist_corners_(undist_corners), bearings_(bearings), rot_cm_(rot_cm), trans_cm_(trans_cm), id_(id) {}

} // namespace data
} // namespace openvslam
