#include "myslam/frame.h"

namespace myslam
{

    Frame::Frame(uint64_t frame_id, double time_stamp, SE3 &pose, cv::Mat &left_img, cv::Mat &right_img)
        : frame_id_(frame_id), time_stamp_(time_stamp), pose_(pose), left_img_(left_img), right_img_(right_img)
    {
    }

    SE3 Frame::getPose()
    {
        std::unique_lock<std::mutex> pose_lock(pose_mutex_);
        return pose_;
    }

    void Frame::setPose(const SE3 &pose)
    {
        std::unique_lock<std::mutex> pose_lock(pose_mutex_);
        pose_ = pose;
    }

    void Frame::setKeyFrame()
    {
        static uint64_t keyframe_id = 0;
        is_keyframe_ = true;
        keyframe_id_ = keyframe_id;
        keyframe_id++;
    }

    Frame::Ptr Frame::createFrame()
    {
        static uint64_t frame_id = 0;
        Frame::Ptr framePtr(new Frame); // 智能指针，本身含有引用计数，一般情况下不用考虑深拷贝构造
        // Frame::Ptr framePtr = new Frame; //裸指针，不能直接将裸指针赋值给智能指针
        framePtr->frame_id_ = frame_id;
        frame_id++;
        return framePtr;
    }
}