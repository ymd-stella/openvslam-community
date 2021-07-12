#include "openvslam/module/marker_detector.h"
#include "openvslam/data/marker2d.h"
#include "openvslam/camera/base.h"
#include "openvslam/solve/pnp_solver.h"

#include <opencv2/highgui.hpp>

namespace openvslam {
namespace module {
marker_detector::marker_detector(const camera::base* camera, const unsigned int num_iter)
    : camera_(camera), num_iter_(num_iter) {
    parameters_ = cv::aruco::DetectorParameters::create();
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
}

void marker_detector::detect(const cv::_InputArray& in_image, std::unordered_map<unsigned int, data::marker2d>& markers_2d) const {
    if (in_image.empty()) {
        return;
    }

    // get cv::Mat of image
    const auto image = in_image.getMat();
    assert(image.type() == CV_8UC1);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary_, corners, ids, parameters_);

    eigen_alloc_vector<Vec3_t> corners_pos;
    const double marker_length = 0.24;
    corners_pos.resize(4);
    corners_pos.at(0) << -marker_length / 2.0, marker_length / 2.0, 0.0;
    corners_pos.at(1) << marker_length / 2.0, marker_length / 2.0, 0.0;
    corners_pos.at(2) << marker_length / 2.0, -marker_length / 2.0, 0.0;
    corners_pos.at(3) << -marker_length / 2.0, -marker_length / 2.0, 0.0;

    for (unsigned int i = 0; i < corners.size(); ++i) {
        std::vector<cv::Point2f> undist_corners;
        camera_->undistort_points(corners[i], undist_corners);
        eigen_alloc_vector<Vec3_t> bearings;
        camera_->convert_points_to_bearings(undist_corners, bearings);

        Mat33_t rot_cm;
        Vec3_t trans_cm;
        const double reproj_error_threashold = 0.1;
        double reproj_error = solve::pnp_solver::compute_pose(bearings, corners_pos, rot_cm, trans_cm, num_iter_);
        if (reproj_error < reproj_error_threashold) {
            markers_2d.emplace(ids[i], data::marker2d(undist_corners, bearings, rot_cm, trans_cm, ids[i]));
        }
    }
}

} // namespace module
} // namespace openvslam
