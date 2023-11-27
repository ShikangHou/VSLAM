#include "myslam/frontend.h"
#include "myslam/config.h"
#include "myslam/algorithm.h"

/* 
前端思路：
初始化。在左相机图像中提取关键点，在左右图像间进行光流追踪，
        此时双目坐标系与世界坐标系重合，相当于知道左相机与右相机的在世界坐标系中的位姿，
        对左右两张图片进行三角化，计算得到特征点对应路标点的世界坐标系位置，初始化成功。
追踪。根据之前两帧图像的位移粗略估计当前相机的位姿，以当前位姿估计上一帧特征点在此帧中的位置，作为LK光流的初始值。
    在前后两帧之间使用LK光流。根据光流追踪的结果优化当前相机的位姿。




 */

namespace myslam
{
    FrontEnd::FrontEnd()
    {
        gftt_ = cv::GFTTDetector::create(Config::get<int>("num_features"), 0.01, 20);
        num_features_init_ = Config::get<int>("num_features_init");
        num_features_ = Config::get<int>("num_features");
    }

    bool FrontEnd::addFrame(Frame::Ptr frame)
    {
        current_frame_ = frame;

        switch (status_)
        {
        case INITING:
            stereoInit();
            break;
        case TRACKING_BAD:
        case TRACKING_GOOD:
            track();
            break;

        case LOST:
            reset();
            break;

        default:
            break;
        }
    }

    bool FrontEnd::stereoInit()
    {
        int num_features_left = detectFeatures();
        int num_coor_features = findFeaturesInRight();
        if (num_coor_features < num_features_init_)
        {
            return false;
        }
        bool build_map_success = buildInitMap();
        if (build_map_success)
        {
            status_ = TRACKING_GOOD;
            // todo
            // viewer
            return true;
        }
        return false;
    }

    bool FrontEnd::track()
    {
        if (last_frame_)
        { // 粗略推算此帧的位姿
            current_frame_->setPose(relative_motion_ * last_frame_->pose_);
        }
        int num_track_last = trackLastFrame();
        // tracking_inliers_ = 
    }

    bool FrontEnd::reset()
    {
    }

    int FrontEnd::detectFeatures()
    {
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255); // 生成一张全白的mask
        for (auto &feat : current_frame_->feature_left_)
        { // 把已有特征点附近涂黑
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10), feat->position_.pt + cv::Point2f(10, 10), cv::Scalar(0), cv::FILLED);
        }

        std::vector<cv::KeyPoint> keypoints;
        gftt_->detect(current_frame_->left_img_, keypoints, mask);
        int cnt_detected = 0;
        for (auto &kp : keypoints)
        {
            current_frame_->feature_left_.push_back(Feature::Ptr(new Feature(current_frame_, kp)));
            cnt_detected++;
        }
        LOG(INFO) << "Detect " << cnt_detected << " new features";
        return cnt_detected;
    }

    int FrontEnd::findFeaturesInRight()
    {
        // use LK flow to estimate points in the right image
        // 先计算出初始值，使光流计算更准确
        std::vector<cv::Point2f> kps_left, kps_right;
        for (auto &kp : current_frame_->feature_left_)
        {
            kps_left.push_back(kp->position_.pt);
            auto map = kp->MapPoint_.lock();
            if (map)
            {
                auto px = camera_right_->world2camera(map->pos_, current_frame_->pose_);
                kps_right.push_back(cv::Point2f(px[0], px[1]));
            }
            else
            {
                kps_right.push_back(kp->position_.pt);
            }
        }

        // LK光流
        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(current_frame_->left_img_, current_frame_->right_img_, kps_left, kps_right,
                                 status, error, cv::Size(11, 11), 3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), // 最大迭代30次，误差阈值0.01
                                 cv::OPTFLOW_USE_INITIAL_FLOW);                                               // 使用OPTFLOW_USE_INITIAL_FLOW，使用kps_right的初始值

        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); i++)
        {
            if (status[i])
            {
                cv::KeyPoint kp(kps_right[i], 7);
                Feature::Ptr feat(new Feature(current_frame_, kp));
                feat->is_on_left_image_ = false;
                current_frame_->feature_right_.push_back(feat);
                num_good_pts++;
            }
            else
            {
                current_frame_->feature_right_.push_back(nullptr);
            }
        }

        LOG(INFO) << "Find " << num_good_pts << " in the right image.";
        return num_good_pts;
    }

    bool FrontEnd::buildInitMap()
    {
        std::vector<SE3> poses{camera_left_->getPose(), camera_right_->getPose()};
        size_t cnt_init_landmarks = 0;
        for (size_t i = 0; i < current_frame_->feature_left_.size(); i++)
        {
            if (current_frame_->feature_right_[i] == nullptr)
                continue;

            std::vector<Vec3> points{
                camera_left_->pixel2camera(Vec2(current_frame_->feature_left_[i]->position_.pt.x,
                                                current_frame_->feature_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(Vec2(current_frame_->feature_right_[i]->position_.pt.x,
                                                 current_frame_->feature_right_[i]->position_.pt.y))};

            Vec3 pworld = Vec3::Zero();

            // 传入的是相机相对于双目坐标系的位姿，因此算得特征点在双目坐标系下的坐标，初始时认为双目坐标系和世界坐标系重合
            if (triangulation(poses, points, pworld) && pworld[2] > 0)
            {
                auto new_map_point = MapPoint::createMapPoint();
                new_map_point->setPos(pworld);
                new_map_point->addObservation(current_frame_->feature_left_[i]);
                new_map_point->addObservation(current_frame_->feature_right_[i]);
                current_frame_->feature_left_[i]->MapPoint_ = new_map_point;
                current_frame_->feature_right_[i]->MapPoint_ = new_map_point;
                cnt_init_landmarks++;
                map_->insertMapPoint(new_map_point);
            }
        }
        LOG(INFO) << "Initial map created with " << cnt_init_landmarks << " map points";
        return true;
    }

    int FrontEnd::trackLastFrame()
    {
        // 给出LK光流追踪的初始值
        std::vector<cv::Point2f> kps_last, kps_current;
        for (auto &kp : last_frame_->feature_left_)
        {
            auto mp = kp->MapPoint_.lock();
            if (mp)
            {
                auto px = camera_left_->world2pixel(mp->getPos(), current_frame_->getPose());
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(cv::Point2f(px.x(), px.y()));
            }
            else
            {
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(last_frame_->left_img_, current_frame_->left_img_, kps_last, kps_current,
                                 status, error, cv::Size(11, 11), 3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01),
                                 cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); i++)
        {
            if (status[i])
            {
                cv::KeyPoint kp(kps_current[i], 7);
                Feature::Ptr feat(new Feature(current_frame_, kp));
                feat->MapPoint_ = last_frame_->feature_left_[i]->MapPoint_;
                current_frame_->feature_left_.push_back(feat);
                num_good_pts++;
            }
        }
        LOG(INFO) << "Find " << num_good_pts << " in the last image.";
        return num_good_pts;
    }

    int FrontEnd::estimateCurrentPose()
    {

    }

}