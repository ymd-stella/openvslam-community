#ifndef OPENVSLAM_IMU_INTERNAL_INERTIAL_EDGE_WRAPPER_H
#define OPENVSLAM_IMU_INTERNAL_INERTIAL_EDGE_WRAPPER_H

#include "openvslam/imu/preintegrated.h"
#include "openvslam/imu/internal/inertial_edge_on_camera.h"
#include "openvslam/imu/internal/inertial_edge_on_imu.h"

#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/base_multi_edge.h>

namespace openvslam {
namespace imu {
namespace internal {

class inertial_edge_wrapper {
public:
    inertial_edge_wrapper() = delete;

    inertial_edge_wrapper(const std::shared_ptr<preintegrated>& imu_preintegrated,
                          bias_vertex* acc_bias_vtx, bias_vertex* gyr_bias_vtx,
                          optimize::internal::se3::shot_vertex* keyfrm_vtx1, velocity_vertex* velocity_vtx1,
                          optimize::internal::se3::shot_vertex* keyfrm_vtx2, velocity_vertex* velocity_vtx2,
                          const float sqrt_chi_sq, const bool use_huber_loss = true,
                          const std::shared_ptr<imu::config>& cfg = nullptr);

    virtual ~inertial_edge_wrapper() = default;

    bool is_inlier() const;

    bool is_outlier() const;

    void set_as_inlier() const;

    void set_as_outlier() const;

    g2o::OptimizableGraph::Edge* edge_;
};

inline inertial_edge_wrapper::inertial_edge_wrapper(const std::shared_ptr<preintegrated>& imu_preintegrated,
                                                    bias_vertex* acc_bias_vtx, bias_vertex* gyr_bias_vtx,
                                                    optimize::internal::se3::shot_vertex* keyfrm_vtx1, velocity_vertex* velocity_vtx1,
                                                    optimize::internal::se3::shot_vertex* keyfrm_vtx2, velocity_vertex* velocity_vtx2,
                                                    const float sqrt_chi_sq, const bool use_huber_loss,
                                                    const std::shared_ptr<imu::config>& cfg) {
    // 拘束条件を設定
    g2o::BaseMultiEdge<9, std::shared_ptr<preintegrated>>* edge;
    if (cfg) {
        edge = new inertial_edge_on_camera(cfg);
    }
    else {
        edge = new inertial_edge_on_imu();
    }

    edge->setInformation(imu_preintegrated->get_information().block<9, 9>(0, 0));
    edge->setMeasurement(imu_preintegrated);

    edge->setVertex(0, keyfrm_vtx1);
    edge->setVertex(1, velocity_vtx1);
    edge->setVertex(2, acc_bias_vtx);
    edge->setVertex(3, gyr_bias_vtx);
    edge->setVertex(4, keyfrm_vtx2);
    edge->setVertex(5, velocity_vtx2);

    edge_ = edge;

    // loss functionを設定
    if (use_huber_loss) {
        auto huber_kernel = new g2o::RobustKernelHuber();
        huber_kernel->setDelta(sqrt_chi_sq);
        edge_->setRobustKernel(huber_kernel);
    }
}

inline bool inertial_edge_wrapper::is_inlier() const {
    return edge_->level() == 0;
}

inline bool inertial_edge_wrapper::is_outlier() const {
    return edge_->level() != 0;
}

inline void inertial_edge_wrapper::set_as_inlier() const {
    edge_->setLevel(0);
}

inline void inertial_edge_wrapper::set_as_outlier() const {
    edge_->setLevel(1);
}

} // namespace internal
} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_INTERNAL_INERTIAL_EDGE_WRAPPER_H
