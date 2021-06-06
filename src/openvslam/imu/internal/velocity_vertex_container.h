#ifndef OPENVSLAM_IMU_INTERNAL_VELOCITY_VERTEX_CONTAINER_H
#define OPENVSLAM_IMU_INTERNAL_VELOCITY_VERTEX_CONTAINER_H

#include "openvslam/type.h"
#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/imu/internal/velocity_vertex.h"

#include <memory>

namespace openvslam {
namespace imu {
namespace internal {

class velocity_vertex_container {
public:
    //! Constructor
    explicit velocity_vertex_container(const std::shared_ptr<unsigned int> offset, const unsigned int num_reserve = 50);

    //! Destructor
    virtual ~velocity_vertex_container() = default;

    //! Create and return the g2o vertex created from the specified frame
    velocity_vertex* create_vertex(data::frame* frm, const bool is_constant = false);

    //! Create and return the g2o vertex created from the specified keyframe
    velocity_vertex* create_vertex(data::keyframe* keyfrm, const bool is_constant = false);

    //! Create and return the g2o vertex created from vertex ID and camera pose
    velocity_vertex* create_vertex(const unsigned int id, const Vec3_t& velocity, const bool is_constant = false);

    //! Get vertex corresponding with the specified frame
    velocity_vertex* get_vertex(data::frame* frm) const;

    //! Get vertex corresponding with the specified keyframe
    velocity_vertex* get_vertex(data::keyframe* keyfrm) const;

    //! Get vertex corresponding with the specified vertex (frame/keyframe) ID
    velocity_vertex* get_vertex(const unsigned int id) const;

    //! Convert frame ID to vertex ID
    unsigned int get_vertex_id(data::frame* frm) const;

    //! Convert keyframe ID to vertex ID
    unsigned int get_vertex_id(data::keyframe* keyfrm) const;

    //! Convert vertex (frame/keyframe) ID to vertex ID
    unsigned int get_vertex_id(unsigned int id) const;

    //! Convert vertex ID to vertex (frame/keyframe) ID
    unsigned int get_id(velocity_vertex* vtx);

    //! Convert vertex ID to vertex (frame/keyframe) ID
    unsigned int get_id(unsigned int vtx_id) const;

    //! Contains the specified keyframe or not
    bool contain(data::keyframe* keyfrm) const;
    bool contain(unsigned int id) const;

    // iterators to sweep vertices
    using iterator = std::unordered_map<unsigned int, velocity_vertex*>::iterator;
    using const_iterator = std::unordered_map<unsigned int, velocity_vertex*>::const_iterator;
    iterator begin();
    const_iterator begin() const;
    iterator end();
    const_iterator end() const;

private:
    const std::shared_ptr<unsigned int> offset_ = nullptr;

    //! key: vertex ID, value: vertex
    std::unordered_map<unsigned int, velocity_vertex*> vtx_container_;

    //! key: frame/keyframe ID, value: vertex ID
    std::unordered_map<unsigned int, unsigned int> vtx_id_container_;

    //! key: vertex ID, value: frame/keyframe ID
    std::unordered_map<unsigned int, unsigned int> id_container_;
};

inline velocity_vertex_container::velocity_vertex_container(const std::shared_ptr<unsigned int> offset, const unsigned int num_reserve)
    : offset_(offset) {
    vtx_container_.reserve(num_reserve);
    vtx_id_container_.reserve(num_reserve);
    id_container_.reserve(num_reserve);
}

inline velocity_vertex* velocity_vertex_container::create_vertex(data::frame* frm, const bool is_constant) {
    return create_vertex(frm->id_, frm->velocity_, is_constant);
}

inline velocity_vertex* velocity_vertex_container::create_vertex(data::keyframe* keyfrm, const bool is_constant) {
    return create_vertex(keyfrm->id_, keyfrm->velocity_, is_constant);
}

inline velocity_vertex* velocity_vertex_container::create_vertex(const unsigned int id,
                                                                 const Vec3_t& velocity,
                                                                 const bool is_constant) {
    // vertexを作成
    const auto vtx_id = *offset_;
    (*offset_)++;
    auto vtx = new velocity_vertex();
    vtx->setId(vtx_id);
    vtx->setEstimate(velocity);
    vtx->setFixed(is_constant);
    // databaseに登録
    id_container_[vtx_id] = id;
    vtx_id_container_[id] = vtx_id;
    vtx_container_[id] = vtx;
    // 作成したvertexをreturn
    return vtx;
}

inline velocity_vertex* velocity_vertex_container::get_vertex(data::frame* frm) const {
    return get_vertex(frm->id_);
}

inline velocity_vertex* velocity_vertex_container::get_vertex(data::keyframe* keyfrm) const {
    return get_vertex(keyfrm->id_);
}

inline velocity_vertex* velocity_vertex_container::get_vertex(const unsigned int id) const {
    return vtx_container_.at(id);
}

inline unsigned int velocity_vertex_container::get_vertex_id(data::frame* frm) const {
    return get_vertex_id(frm->id_);
}

inline unsigned int velocity_vertex_container::get_vertex_id(data::keyframe* keyfrm) const {
    return get_vertex_id(keyfrm->id_);
}

inline unsigned int velocity_vertex_container::get_vertex_id(unsigned int id) const {
    return vtx_id_container_.at(id);
}

inline unsigned int velocity_vertex_container::get_id(velocity_vertex* vtx) {
    return get_id(vtx->id());
}

inline unsigned int velocity_vertex_container::get_id(unsigned int vtx_id) const {
    return id_container_.at(vtx_id);
}

inline bool velocity_vertex_container::contain(data::keyframe* keyfrm) const {
    return 0 != vtx_container_.count(keyfrm->id_);
}

inline bool velocity_vertex_container::contain(unsigned int id) const {
    return 0 != vtx_container_.count(id);
}

inline velocity_vertex_container::iterator velocity_vertex_container::begin() {
    return vtx_container_.begin();
}

inline velocity_vertex_container::const_iterator velocity_vertex_container::begin() const {
    return vtx_container_.begin();
}

inline velocity_vertex_container::iterator velocity_vertex_container::end() {
    return vtx_container_.end();
}

inline velocity_vertex_container::const_iterator velocity_vertex_container::end() const {
    return vtx_container_.end();
}

} // namespace internal
} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_INTERNAL_VELOCITY_VERTEX_CONTAINER_H
