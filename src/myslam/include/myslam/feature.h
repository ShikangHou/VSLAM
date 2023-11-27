#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include "myslam/common_include.h"
#include <opencv2/opencv.hpp>

// Frame Feature Mappoint的关系：
// 一帧图像Frame中含有很多特征点Feature，多个特征点对应同一个路标Mappoint

// 将Feature看作Frame 和 Mappoint 的桥梁，当作子类

namespace myslam
{
    // 前置声明，解决头文件相互包含的问题
    class Frame;
    class MapPoint;

    class Feature
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Feature> Ptr;

        std::weak_ptr<Frame> frame_; // 使用weak_ptr,weak_ptr不会引用计数，解决share_ptr互相引用的问题
        cv::KeyPoint position_;
        std::weak_ptr<MapPoint> MapPoint_; // 使用weak_ptr 解决share_ptr互相引用的问题

        bool is_outlier_ = false;
        bool is_on_left_image_ = true;

    public:
        Feature(){};
        Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp) // 变量为weak_ptr类型，但是赋值仍然用share_ptr
            : frame_(frame), position_(kp)
        {
        }
    };

}

#endif