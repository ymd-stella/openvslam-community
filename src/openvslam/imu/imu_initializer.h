#ifndef OPENVSLAM_OPTIMIZE_IMU_INITIALIZER_H
#define OPENVSLAM_OPTIMIZE_IMU_INITIALIZER_H

namespace openvslam {

namespace data {
class keyframe;
} // namespace data

namespace optimize {

class imu_initializer {
public:
    /**
     * Constructor
     * @param num_iter
     */
    explicit imu_initializer(const unsigned int num_iter = 10);

    /**
     * Destructor
     */
    virtual ~imu_initializer() = default;

    /**
     * Perform optimization
     */
    bool initialize(const std::vector<data::keyframe*>& keyfrms, Mat33_t& Rwg, double& scale,
                    bool depth_is_avaliable, float info_prior_gyr, float info_prior_acc) const;

private:
    //! number of iterations of optimization
    unsigned int num_iter_;
};

} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZE_IMU_INITIALIZER_H
