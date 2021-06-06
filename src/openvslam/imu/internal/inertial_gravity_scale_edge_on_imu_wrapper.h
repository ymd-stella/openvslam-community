#ifndef OPENVSLAM_IMU_INTERNAL_GRAVITY_SCALE_EDGE_ON_IMU_WRAPPER_H
#define OPENVSLAM_IMU_INTERNAL_GRAVITY_SCALE_EDGE_ON_IMU_WRAPPER_H

#include "openvslam/imu/preintegrated.h"
#include "openvslam/imu/internal/inertial_gravity_scale_edge_on_imu.h"

#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/base_multi_edge.h>

namespace openvslam {
namespace imu {
namespace internal {

class inertial_gravity_scale_edge_on_imu_wrapper {
public:
    inertial_gravity_scale_edge_on_imu_wrapper() = delete;

    inertial_gravity_scale_edge_on_imu_wrapper(const std::shared_ptr<preintegrated>& imu_preintegrated,
                                               bias_vertex* acc_bias_vtx, bias_vertex* gyr_bias_vtx,
                                               optimize::internal::se3::shot_vertex* keyfrm_vtx1, velocity_vertex* velocity_vtx1,
                                               optimize::internal::se3::shot_vertex* keyfrm_vtx2, velocity_vertex* velocity_vtx2,
                                               gravity_dir_vertex* gravity_dir_vtx, scale_vertex* scale_vtx);

    virtual ~inertial_gravity_scale_edge_on_imu_wrapper() = default;

    bool is_inlier() const;

    bool is_outlier() const;

    void set_as_inlier() const;

    void set_as_outlier() const;

    g2o::OptimizableGraph::Edge* edge_;
};

inline inertial_gravity_scale_edge_on_imu_wrapper::inertial_gravity_scale_edge_on_imu_wrapper(
    const std::shared_ptr<preintegrated>& imu_preintegrated,
    bias_vertex* acc_bias_vtx, bias_vertex* gyr_bias_vtx,
    optimize::internal::se3::shot_vertex* keyfrm_vtx1,
    velocity_vertex* velocity_vtx1,
    optimize::internal::se3::shot_vertex* keyfrm_vtx2,
    velocity_vertex* velocity_vtx2,
    gravity_dir_vertex* gravity_dir_vtx,
    scale_vertex* scale_vtx) {
    g2o::BaseMultiEdge<9, std::shared_ptr<preintegrated>>* edge;
    edge = new inertial_gravity_scale_edge_on_imu();

    edge->setInformation(imu_preintegrated->get_information().block<9, 9>(0, 0));
    edge->setMeasurement(imu_preintegrated);

    edge->setVertex(0, keyfrm_vtx1);
    edge->setVertex(1, velocity_vtx1);
    edge->setVertex(2, gyr_bias_vtx);
    edge->setVertex(3, acc_bias_vtx);
    edge->setVertex(4, keyfrm_vtx2);
    edge->setVertex(5, velocity_vtx2);
    edge->setVertex(6, gravity_dir_vtx);
    edge->setVertex(7, scale_vtx);

    edge_ = edge;
}

inline bool inertial_gravity_scale_edge_on_imu_wrapper::is_inlier() const {
    return edge_->level() == 0;
}

inline bool inertial_gravity_scale_edge_on_imu_wrapper::is_outlier() const {
    return edge_->level() != 0;
}

inline void inertial_gravity_scale_edge_on_imu_wrapper::set_as_inlier() const {
    edge_->setLevel(0);
}

inline void inertial_gravity_scale_edge_on_imu_wrapper::set_as_outlier() const {
    edge_->setLevel(1);
}

} // namespace internal
} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_INTERNAL_GRAVITY_SCALE_EDGE_ON_IMU_WRAPPER_H
