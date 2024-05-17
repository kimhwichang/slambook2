//
// Created by gaoxiang on 19-5-2.
//
#pragma once

#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>
#include "myslam/common_include.h"

namespace myslam {

struct Frame;
struct MapPoint;

struct Feature {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;         // 이 feature 가 속한 frame
    cv::KeyPoint     position_;              // 2D pixel 위치
    std::weak_ptr<MapPoint> map_point_;  // 맵 포인트 연결

    bool is_outlier_ = false;       // 이상점 여부
    bool is_on_left_image_ = true;  // 왼쪽 그림에 제시할지 ?  <-> false 면 오른쪽 그림

   public:
    Feature() {}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), position_(kp) {}
};
}  // namespace myslam

#endif  // MYSLAM_FEATURE_H
