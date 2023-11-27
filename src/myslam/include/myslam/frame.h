#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "myslam/common_include.h"
#include <mutex>
#include <vector>
#include <opencv2/opencv.hpp>

// Frame Feature Mappoint的关系：
// 一帧图像Frame中含有很多特征点Feature，多个特征点对应同一个路标Mappoint

namespace myslam
{
    // 前置声明，解决头文件互相调用问题
    class Feature;
    class MapPoint;

    class Frame
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // EIGEN 地址对齐

        typedef std::shared_ptr<Frame> Ptr; // 成员变量使用智能指针，本身含有引用计数，自动管理生命周期，
                                            // 一般情况下不需要实现深拷贝构造

        uint64_t frame_id_ = 0;    // id of this frame
        uint64_t keyframe_id_ = 0; // if of key frame
        bool is_keyframe_ = false; // 是否为关键帧
        double time_stamp_;        // 时间戳
        SE3 pose_;                 // Tcw形式 pose 即world坐标系在camera坐标系下的表达
        std::mutex pose_mutex_;    // 锁
        cv::Mat left_img_, right_img_;

        std::vector<std::shared_ptr<Feature>> feature_left_;
        std::vector<std::shared_ptr<Feature>> feature_right_;

    public:
        Frame(){};
        Frame(uint64_t id, double time_stamp, SE3 &pose, cv::Mat &left_img, cv::Mat &right_img);

        // 返回位姿
        SE3 getPose();
        // 设置位姿
        void setPose(const SE3 &pose);

        // 设置当前帧为关键帧，并自动更新关键帧id
        void setKeyFrame();
        // 创建一个Frame，并自动更新帧id
        Ptr createFrame();
    };
}

#endif