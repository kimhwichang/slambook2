#pragma once
#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam {
class Map {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType; // map point 
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType; // key frame

    Map() {}

    ///키프레임 추가
    void InsertKeyFrame(Frame::Ptr frame);
    /// 지도에 꼭지점 추가
    void InsertMapPoint(MapPoint::Ptr map_point);

    /// 모든 지도점 가져오기
    LandmarksType GetAllMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }
    /// 모든 키프레임 가져오기
    KeyframesType GetAllKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    /// 활성 맵 가져오기
    LandmarksType GetActiveMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    /// 활성 키프레임 가져오기
    KeyframesType GetActiveKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    /// 清理map中观测数量为零的点
    void CleanMap();

   private:
    // 将旧的关键帧置为不活跃状态
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;         // all landmarks
    LandmarksType active_landmarks_;  // active landmarks
    KeyframesType keyframes_;         // all key-frames
    KeyframesType active_keyframes_;  // all key-frames

    Frame::Ptr current_frame_ = nullptr;

    // settings
    int num_active_keyframes_ = 7;  //최신 7개의 키프레임만 유지
};
}  // namespace myslam

#endif  // MAP_H
