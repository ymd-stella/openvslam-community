#ifndef OPENVSLAM_IMU_INTERNAL_BIAS_VERTEX_H
#define OPENVSLAM_IMU_INTERNAL_BIAS_VERTEX_H

#include "openvslam/type.h"

#include <g2o/core/base_vertex.h>

namespace openvslam {
namespace imu {
namespace internal {

class bias_vertex final : public g2o::BaseVertex<3, Vec3_t> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bias_vertex();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void setToOriginImpl() override;

    void oplusImpl(const number_t* update_) override;
};

inline bias_vertex::bias_vertex()
    : g2o::BaseVertex<3, Vec3_t>() {}

inline bool bias_vertex::read(std::istream& is) {
    Vec3_t est;
    read_matrix(is, est);
    setEstimate(est);
    return true;
}

inline bool bias_vertex::write(std::ostream& os) const {
    write_matrix(os, estimate());
    return os.good();
}

inline void bias_vertex::setToOriginImpl() {
    setEstimate(Vec3_t::Zero());
}

inline void bias_vertex::oplusImpl(const number_t* update_) {
    Eigen::Map<const Vec3_t> update(update_);
    setEstimate(estimate() + update);
}

} // namespace internal
} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_INTERNAL_BIAS_VERTEX_H
