#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/camera.h"

namespace myslam
{
    typedef enum
    {
        INITING,
        TRACKING_GOOD,
        TRACKING_BAD,
        LOST
    } FrontendStatus;

    class FrontEnd
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<FrontEnd> Ptr;

        FrontEnd();

        bool addFrame(Frame::Ptr frame);

    private:
        bool stereoInit();
        
        bool track();

        bool reset();

        int detectFeatures();
        int findFeaturesInRight();
        bool buildInitMap();

        int trackLastFrame();
        int estimateCurrentPose();
        
        FrontendStatus status_ = INITING;
        Frame::Ptr current_frame_ = nullptr; // 当前帧
        Frame::Ptr last_frame_ = nullptr;    // 上一帧
        Camera::Ptr camera_left_ = nullptr;
        Camera::Ptr camera_right_ = nullptr;

        Map::Ptr map_ = nullptr;

        SE3 relative_motion_; // 当前帧与上一帧的相对运动，用于估计当前帧pose初值

        int tracking_inliers_ = 0;

        // params
        int num_features_ = 200;
        int num_features_init_ = 100;
        int num_features_tracking_ = 50;
        int num_features_tracking_bad_ = 20;
        int num_features_needed_for_keyframe_ = 80;

        cv::Ptr<cv::GFTTDetector> gftt_; // feature detector in opencv
    };

}

#endif
