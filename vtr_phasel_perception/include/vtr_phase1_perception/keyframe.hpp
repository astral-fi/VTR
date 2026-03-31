#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <DBoW2/DBoW2.h>
#include <ros/time.h>

namespace vtr_phase1 {

struct Feature2D {
    float u, v;           // pixel coords
    cv::Mat descriptor;   // 32-byte BRIEF descriptor (CV_8UC1, 1x32)
};

struct Feature3D {
    float x, y, z;        // 3D point in left camera frame
};

struct Keyframe {
    uint32_t id;
    ros::Time timestamp;

    // T_{i-1}^{i}: relative transform from previous keyframe to this one
    Eigen::Matrix4d T_rel;

    std::vector<Feature2D> features_2d;
    std::vector<Feature3D> features_3d;  // paired 1:1 with features_2d

    DBoW2::BowVector bow_vector;

    cv::Mat image;  // CV_8UC1 grayscale

    bool is_keyframe = true;
};

} 