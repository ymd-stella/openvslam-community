#ifndef OPENVSLAM_IMU_INTERNAL_PRIOR_BIAS_EDGE_WRAPPER_H
#define OPENVSLAM_IMU_INTERNAL_PRIOR_BIAS_EDGE_WRAPPER_H

#include "openvslam/imu/internal/prior_bias_edge.h"

namespace openvslam {
namespace imu {
namespace internal {

class prior_bias_edge_wrapper {
public:
    prior_bias_edge_wrapper() = delete;

    prior_bias_edge_wrapper(double info_prior, bias_vertex* bias_vtx);

    virtual ~prior_bias_edge_wrapper() = default;

    bool is_inlier() const;

    bool is_outlier() const;

    void set_as_inlier() const;

    void set_as_outlier() const;

    g2o::OptimizableGraph::Edge* edge_;
};

inline prior_bias_edge_wrapper::prior_bias_edge_wrapper(double info_prior, bias_vertex* bias_vtx) {
    auto edge = new prior_bias_edge();
    edge->setInformation(info_prior * Mat33_t::Identity());
    edge->setMeasurement(Vec3_t::Zero());
    edge->setVertex(0, bias_vtx);
    edge_ = edge;
}

inline bool prior_bias_edge_wrapper::is_inlier() const {
    return edge_->level() == 0;
}

inline bool prior_bias_edge_wrapper::is_outlier() const {
    return edge_->level() != 0;
}

inline void prior_bias_edge_wrapper::set_as_inlier() const {
    edge_->setLevel(0);
}

inline void prior_bias_edge_wrapper::set_as_outlier() const {
    edge_->setLevel(1);
}

} // namespace internal
} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_INTERNAL_PRIOR_BIAS_EDGE_WRAPPER_H
