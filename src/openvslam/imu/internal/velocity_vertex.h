#ifndef OPENVSLAM_IMU_INTERNAL_VELOCITY_VERTEX_H
#define OPENVSLAM_IMU_INTERNAL_VELOCITY_VERTEX_H

#include "openvslam/type.h"

#include <g2o/core/base_vertex.h>

namespace openvslam {
namespace imu {
namespace internal {

class velocity_vertex final : public g2o::BaseVertex<3, Vec3_t> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    velocity_vertex();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void setToOriginImpl() override;

    void oplusImpl(const number_t* update_) override;
};

inline velocity_vertex::velocity_vertex()
    : g2o::BaseVertex<3, Vec3_t>() {}

inline bool velocity_vertex::read(std::istream& is) {
    Vec3_t est;
    read_matrix(is, est);
    setEstimate(est);
    return true;
}

inline bool velocity_vertex::write(std::ostream& os) const {
    const auto est = estimate();
    for (unsigned int i = 0; i < est.size(); ++i) {
        os << est[i] << " ";
    }
    return os.good();
}

inline void velocity_vertex::setToOriginImpl() {
    setEstimate(Vec3_t::Zero());
}

inline void velocity_vertex::oplusImpl(const number_t* update_) {
    Eigen::Map<const Vec3_t> update(update_);
    setEstimate(estimate() + update);
}

} // namespace internal
} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_INTERNAL_VELOCITY_VERTEX_H
