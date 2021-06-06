#ifndef OPENVSLAM_IMU_INTERNAL_BIAS_VERTEX_CONTAINER_H
#define OPENVSLAM_IMU_INTERNAL_BIAS_VERTEX_CONTAINER_H

#include "openvslam/type.h"
#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/imu/internal/bias_vertex.h"

#include <memory>

namespace openvslam {
namespace imu {
namespace internal {

class bias_vertex_container {
public:
    //! Constructor
    explicit bias_vertex_container(const std::shared_ptr<unsigned int> offset, const unsigned int num_reserve = 50);

    //! Destructor
    virtual ~bias_vertex_container() = default;

    //! Create and return the g2o vertex created from vertex ID and camera pose
    bias_vertex* create_vertex(const unsigned int id, const Vec3_t& bias, const bool is_constant = false);

    //! Get vertex corresponding with the specified frame
    bias_vertex* get_vertex(data::frame* frm) const;

    //! Get vertex corresponding with the specified keyframe
    bias_vertex* get_vertex(data::keyframe* keyfrm) const;

    //! Get vertex corresponding with the specified vertex (frame/keyframe) ID
    bias_vertex* get_vertex(const unsigned int id) const;

    //! Convert frame ID to vertex ID
    unsigned int get_vertex_id(data::frame* frm) const;

    //! Convert keyframe ID to vertex ID
    unsigned int get_vertex_id(data::keyframe* keyfrm) const;

    //! Convert vertex (frame/keyframe) ID to vertex ID
    unsigned int get_vertex_id(unsigned int id) const;

    //! Convert vertex ID to vertex (frame/keyframe) ID
    unsigned int get_id(bias_vertex* vtx);

    //! Convert vertex ID to vertex (frame/keyframe) ID
    unsigned int get_id(unsigned int vtx_id) const;

    //! Contains the specified keyframe or not
    bool contain(data::keyframe* keyfrm) const;

    // iterators to sweep vertices
    using iterator = std::unordered_map<unsigned int, bias_vertex*>::iterator;
    using const_iterator = std::unordered_map<unsigned int, bias_vertex*>::const_iterator;
    iterator begin();
    const_iterator begin() const;
    iterator end();
    const_iterator end() const;

private:
    //! Counter to assign a unique ID
    const std::shared_ptr<unsigned int> offset_ = nullptr;

    //! key: vertex ID, value: vertex
    std::unordered_map<unsigned int, bias_vertex*> vtx_container_;

    //! key: frame/keyframe ID, value: vertex ID
    std::unordered_map<unsigned int, unsigned int> vtx_id_container_;

    //! key: vertex ID, value: frame/keyframe ID
    std::unordered_map<unsigned int, unsigned int> id_container_;
};

inline bias_vertex_container::bias_vertex_container(const std::shared_ptr<unsigned int> offset, const unsigned int num_reserve)
    : offset_(offset) {
    vtx_container_.reserve(num_reserve);
    vtx_id_container_.reserve(num_reserve);
    id_container_.reserve(num_reserve);
}

inline bias_vertex* bias_vertex_container::create_vertex(const unsigned int id,
                                                         const Vec3_t& bias,
                                                         const bool is_constant) {
    // Create vertex
    const auto vtx_id = *offset_;
    (*offset_)++;
    auto vtx = new bias_vertex();
    vtx->setId(vtx_id);
    vtx->setEstimate(bias);
    vtx->setFixed(is_constant);
    // Add to database
    id_container_[vtx_id] = id;
    vtx_id_container_[id] = vtx_id;
    vtx_container_[id] = vtx;
    // Return the vertex
    return vtx;
}

inline bias_vertex* bias_vertex_container::get_vertex(data::frame* frm) const {
    return get_vertex(frm->id_);
}

inline bias_vertex* bias_vertex_container::get_vertex(data::keyframe* keyfrm) const {
    return get_vertex(keyfrm->id_);
}

inline bias_vertex* bias_vertex_container::get_vertex(const unsigned int id) const {
    return vtx_container_.at(id);
}

inline unsigned int bias_vertex_container::get_vertex_id(data::frame* frm) const {
    return get_vertex_id(frm->id_);
}

inline unsigned int bias_vertex_container::get_vertex_id(data::keyframe* keyfrm) const {
    return get_vertex_id(keyfrm->id_);
}

inline unsigned int bias_vertex_container::get_vertex_id(unsigned int id) const {
    return vtx_id_container_.at(id);
}

inline unsigned int bias_vertex_container::get_id(bias_vertex* vtx) {
    return get_id(vtx->id());
}

inline unsigned int bias_vertex_container::get_id(unsigned int vtx_id) const {
    return id_container_.at(vtx_id);
}

inline bool bias_vertex_container::contain(data::keyframe* keyfrm) const {
    return 0 != vtx_container_.count(keyfrm->id_);
}

inline bias_vertex_container::iterator bias_vertex_container::begin() {
    return vtx_container_.begin();
}

inline bias_vertex_container::const_iterator bias_vertex_container::begin() const {
    return vtx_container_.begin();
}

inline bias_vertex_container::iterator bias_vertex_container::end() {
    return vtx_container_.end();
}

inline bias_vertex_container::const_iterator bias_vertex_container::end() const {
    return vtx_container_.end();
}

} // namespace internal
} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_INTERNAL_BIAS_VERTEX_CONTAINER_H
