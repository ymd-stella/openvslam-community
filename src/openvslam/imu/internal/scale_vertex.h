#ifndef OPENVSLAM_IMU_INTERNAL_SCALE_VERTEX_H
#define OPENVSLAM_IMU_INTERNAL_SCALE_VERTEX_H

#include "openvslam/type.h"
#include "openvslam/util/converter.h"

#include <g2o/core/base_vertex.h>

namespace openvslam {
namespace imu {
namespace internal {

class scale_vertex final : public g2o::BaseVertex<1, double> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    scale_vertex();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void setToOriginImpl() override;

    void oplusImpl(const number_t* update_) override;
};

inline scale_vertex::scale_vertex()
    : g2o::BaseVertex<1, double>() {}

inline bool scale_vertex::read(std::istream& is) {
    double est;
    is >> est;
    setEstimate(est);
    return true;
}

inline bool scale_vertex::write(std::ostream& os) const {
    os << estimate();
    return os.good();
}

inline void scale_vertex::setToOriginImpl() {
    setEstimate(1.0);
}

inline void scale_vertex::oplusImpl(const number_t* update_) {
    setEstimate(estimate() * std::exp(*update_));
}

} // namespace internal
} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_INTERNAL_SCALE_VERTEX_H
