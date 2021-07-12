#include "openvslam/data/marker.h"

namespace openvslam {
namespace data {

marker::marker(const eigen_alloc_vector<Vec3_t>& corners_pos_w, unsigned int id)
    : corners_pos_w_(corners_pos_w), id_(id) {}

void marker::set_corner_pos(const Vec3_t& pos, unsigned int corner_id) {
    std::lock_guard<std::mutex> lock(mtx_position_);
    corners_pos_w_[corner_id] = pos;
}

} // namespace data
} // namespace openvslam
