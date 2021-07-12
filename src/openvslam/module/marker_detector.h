#ifndef OPENVSLAM_DETECTOR_MARKER_DETECTOR_H
#define OPENVSLAM_DETECTOR_MARKER_DETECTOR_H

#include <unordered_map>
#include <opencv2/aruco.hpp>

namespace openvslam {

namespace data {
class marker2d;
} // namespace data

namespace camera {
class base;
} // namespace camera

namespace module {

class marker_detector {
public:
    //! Constructor
    marker_detector(const camera::base* camera, unsigned int num_iter = 10);

    //! Detect markers
    void detect(const cv::_InputArray& in_image, std::unordered_map<unsigned int, data::marker2d>& markers_2d) const;

    //! camera
    const camera::base* camera_;

    //! iterations of pnp_solver
    unsigned int num_iter_ = 10;

    //! parameters for marker detection
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
};

} // namespace module
} // namespace openvslam

#endif // OPENVSLAM_DETECTOR_MARKER_DETECTOR_H