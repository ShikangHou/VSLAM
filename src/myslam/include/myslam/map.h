#ifndef MYSLAM_MAP_H
#define MYSLAM_MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/feature.h"
#include "myslam/mappoint.h"
#include <unordered_map>

/* 地图：前端和后端的桥梁
前端调用 insertKeyFrame 和 insertMapPoint插入新帧和地图点
后端维护地图的结构outlier/剔除等等
*/

namespace myslam
{
    class Map
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Map> Ptr;
        typedef std::unordered_map<uint16_t, MapPoint::Ptr> LandmarksType;
        typedef std::unordered_map<uint16_t, Frame::Ptr> KeyframeType;

        Map(){};

        // 增加一个关键帧
        void insertKeyFrame(Frame::Ptr frame);
        // 增加一个地图点
        void insertMapPoint(MapPoint::Ptr map_point);

        // 获得所有地图点
        LandmarksType getAllMapPoints()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return landmarks_;
        }

        // 获得活跃地图点
        LandmarksType getActiveMapPoints()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_landmarks_;
        }

        // 获得所有关键帧
        KeyframeType getAllKeyFrame()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return keyframe_;
        }

        // 获得活跃关键帧
        KeyframeType getActiveKeyFrame()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_keyframe_;
        }

        void cleanMap(); // 清理map中观测数量为0的点

    private:
        void removeOldFrame(); // 从活跃的关键帧中去除旧的帧

        std::mutex data_mutex_;          // 数据锁
        LandmarksType landmarks_;        // 所有的路标点
        LandmarksType active_landmarks_; // 活跃的路标点
        KeyframeType keyframe_;          // 所有关键帧
        KeyframeType active_keyframe_;   // 活跃的关键帧

        Frame::Ptr current_frame_ = nullptr;

        int num_activate_keyframes_ = 7; // 激活的关键帧数量
    };
}

#endif
