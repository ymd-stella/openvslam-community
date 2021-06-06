#ifndef OPENVSLAM_IMU_INTERNAL_GRAVITY_DIR_VERTEX_H
#define OPENVSLAM_IMU_INTERNAL_GRAVITY_DIR_VERTEX_H

#include "openvslam/type.h"
#include "openvslam/util/converter.h"

#include <g2o/core/base_vertex.h>

namespace openvslam {
namespace imu {
namespace internal {

class gravity_dir_vertex final : public g2o::BaseVertex<2, Mat33_t> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    gravity_dir_vertex();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void setToOriginImpl() override;

    void oplusImpl(const number_t* update_) override;
};

inline gravity_dir_vertex::gravity_dir_vertex()
    : g2o::BaseVertex<2, Mat33_t>() {}

inline bool gravity_dir_vertex::read(std::istream& is) {
    Mat33_t est;
    read_matrix(is, est);
    setEstimate(est);
    return true;
}

inline bool gravity_dir_vertex::write(std::ostream& os) const {
    write_matrix(os, estimate());
    return os.good();
}

inline void gravity_dir_vertex::setToOriginImpl() {
    setEstimate(Mat33_t::Identity());
}

inline void gravity_dir_vertex::oplusImpl(const number_t* update_) {
    setEstimate(estimate() * util::converter::exp_so3((Vec3_t() << update_[0], update_[1], 0.0).finished()));
}

} // namespace internal
} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_INTERNAL_GRAVITY_DIR_VERTEX_H
