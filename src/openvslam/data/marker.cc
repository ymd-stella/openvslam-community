#include "openvslam/data/marker.h"

namespace openvslam {
namespace data {

marker::marker(const eigen_alloc_vector<Vec3_t>& corners_pos_w, unsigned int id)
    : corners_pos_w_(corners_pos_w), id_(id) {}

} // namespace data
} // namespace openvslam
