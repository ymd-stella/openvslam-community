#ifndef OPENVSLAM_IMU_INTERNAL_PRIOR_BIAS_EDGE_H
#define OPENVSLAM_IMU_INTERNAL_PRIOR_BIAS_EDGE_H

#include <g2o/core/base_unary_edge.h>

namespace openvslam {
namespace imu {
namespace internal {

class prior_bias_edge final : public g2o::BaseUnaryEdge<3, Vec3_t, bias_vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    prior_bias_edge();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void computeError() override;

    void linearizeOplus() override;
};

inline prior_bias_edge::prior_bias_edge()
    : g2o::BaseUnaryEdge<3, Vec3_t, bias_vertex>() {
}

inline bool prior_bias_edge::read(std::istream& is) {
    read_matrix(is, _measurement);
    for (unsigned int i = 0; i < Dimension; ++i) {
        for (unsigned int j = i; j < Dimension; ++j) {
            is >> information()(i, j);
            if (i != j) {
                information()(j, i) = information()(i, j);
            }
        }
    }
    return true;
}

inline bool prior_bias_edge::write(std::ostream& os) const {
    write_matrix(os, _measurement);
    for (unsigned int i = 0; i < Dimension; ++i) {
        for (unsigned int j = i; j < Dimension; ++j) {
            os << " " << information()(i, j);
        }
    }
    return os.good();
}

inline void prior_bias_edge::computeError() {
    const auto bias_vtx = static_cast<const bias_vertex*>(_vertices[0]);
    _error = _measurement - bias_vtx->estimate();
}

inline void prior_bias_edge::linearizeOplus() {
    _jacobianOplusXi.block<3, 3>(0, 0) = Mat33_t::Identity();
}

} // namespace internal
} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_INTERNAL_PRIOR_BIAS_EDGE_H
