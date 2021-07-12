#ifndef OPENVSLAM_DATA_MARKER_H
#define OPENVSLAM_DATA_MARKER_H

#include "openvslam/type.h"
#include <opencv2/core/core.hpp>
#include <Eigen/Core>

namespace openvslam {
namespace data {

class keyframe;

class marker {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! constructor
    marker(const eigen_alloc_vector<Vec3_t>& corners_pos_w, unsigned int id);

    //! corner positions
    eigen_alloc_vector<Vec3_t> corners_pos_w_;

    //! marker ID
    unsigned int id_;

    //! observed keyframes
    std::vector<keyframe*> observations_;
};

} // namespace data
} // namespace openvslam

#endif // OPENVSLAM_DATA_MARKER_H
