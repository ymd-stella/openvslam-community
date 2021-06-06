#include "euroc_util.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cassert>
#include <algorithm>

euroc_sequence::euroc_sequence(const std::string& seq_dir_path)
    : seq_dir_path_(seq_dir_path) {
    const std::string timestamp_file_path = seq_dir_path + "/cam0/data.csv";
    const std::string left_img_dir_path = seq_dir_path + "/cam0/data/";
    const std::string right_img_dir_path = seq_dir_path + "/cam1/data/";

    timestamps_.clear();
    left_img_file_paths_.clear();
    right_img_file_paths_.clear();

    // load timestamps
    std::ifstream ifs_timestamp;
    ifs_timestamp.open(timestamp_file_path.c_str());
    if (!ifs_timestamp) {
        throw std::runtime_error("Could not load a timestamp file from " + timestamp_file_path);
    }

    // load header row
    std::string s;
    getline(ifs_timestamp, s);

    while (!ifs_timestamp.eof()) {
        getline(ifs_timestamp, s);
        std::replace(s.begin(), s.end(), ',', ' ');
        if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            unsigned long long timestamp;
            ss >> timestamp;
            timestamps_.push_back(timestamp / static_cast<double>(1E9));
            left_img_file_paths_.push_back(left_img_dir_path + std::to_string(timestamp) + ".png");
            right_img_file_paths_.push_back(right_img_dir_path + std::to_string(timestamp) + ".png");
        }
    }

    ifs_timestamp.close();
}

std::vector<euroc_sequence::frame> euroc_sequence::get_frames() const {
    std::vector<frame> frames;
    assert(timestamps_.size() == left_img_file_paths_.size());
    assert(timestamps_.size() == right_img_file_paths_.size());
    assert(left_img_file_paths_.size() == right_img_file_paths_.size());
    for (unsigned int i = 0; i < timestamps_.size(); ++i) {
        frames.emplace_back(frame{left_img_file_paths_.at(i), right_img_file_paths_.at(i), timestamps_.at(i)});
    }
    return frames;
}

std::queue<std::shared_ptr<openvslam::imu::data>> euroc_sequence::get_imu_data() const {
    std::queue<std::shared_ptr<openvslam::imu::data>> imu_data;
    const std::string imu_data_file_path = seq_dir_path_ + "/imu0/data.csv";

    // load imu
    std::ifstream ifs_imu_data;
    ifs_imu_data.open(imu_data_file_path.c_str());
    if (!ifs_imu_data) {
        throw std::runtime_error("Could not load a imu data file from " + imu_data_file_path);
    }

    // load header row
    std::string s;
    getline(ifs_imu_data, s);

    while (!ifs_imu_data.eof()) {
        getline(ifs_imu_data, s);
        std::replace(s.begin(), s.end(), ',', ' ');
        if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            unsigned long long timestamp_nsec;
            double wx, wy, wz, ax, ay, az;
            ss >> timestamp_nsec >> wx >> wy >> wz >> ax >> ay >> az;
            double timestamp = timestamp_nsec / static_cast<double>(1E9);
            imu_data.push(std::make_shared<openvslam::imu::data>(ax, ay, az, wx, wy, wz, timestamp));
        }
    }

    ifs_imu_data.close();

    return imu_data;
}
