#include "myslam/map.h"

namespace myslam
{
    void Map::insertKeyFrame(Frame::Ptr frame)
    {
        std::unique_lock<std::mutex> data_lock(data_mutex_);

        current_frame_ = frame;
        if (keyframe_.find(frame->frame_id_) == keyframe_.end()) // 如果keyframe_中没有这一帧，使用insert插入
        {
            keyframe_.insert(std::make_pair(frame->frame_id_, frame));
            active_keyframe_.insert(std::make_pair(frame->frame_id_, frame));
        }
        else // 否则，直接替换为新的帧
        {
            keyframe_[frame->frame_id_] = frame;
            active_keyframe_[frame->frame_id_] = frame;
        }

        if (active_keyframe_.size() > num_activate_keyframes_) // 如果活跃的帧超过了设置的数量，则删除最老的一帧
        {
            removeOldFrame();
        }
    }

    void Map::insertMapPoint(MapPoint::Ptr map_point)
    {
        std::unique_lock<std::mutex> data_lock(data_mutex_);

        if(landmarks_.find(map_point->id_) == landmarks_.end())
        {
            landmarks_.insert(std::make_pair(map_point->id_,map_point));
            active_landmarks_.insert(std::make_pair(map_point->id_,map_point));
        }
        else
        {
            landmarks_[map_point->id_] = map_point;
            active_landmarks_[map_point->id_] = map_point;
        }

    }

    void Map::removeOldFrame()
    {
        if (current_frame_ == nullptr)
            return; // current_frame_ 为空指针说明还没有添加新的帧

        double max_dis = 0, min_dis = 9999;
        double max_kf_id = 0, min_kf_id = 0;
        auto Twc = current_frame_->pose_.inverse();
        for (auto &kf : active_keyframe_)
        {
            if (kf.second == current_frame_)
                continue;
            auto dis = (kf.second->pose_ * Twc).log().norm(); // norm求范数，即距离
            if (dis > max_dis)
            {
                max_dis = dis;
                max_kf_id = kf.first;
            }
            if (dis < min_dis)
            {
                min_dis = dis;
                min_kf_id = kf.first;
            }
        }

        const double min_dis_th = 0.2; // 最近阈值
        Frame::Ptr frame_to_remove = nullptr;
        if (min_dis < min_dis_th)
        {
            // 如果有一帧和当前帧很近，那么删除这一帧
            frame_to_remove = keyframe_.at(min_kf_id);
        }
        else
        { // 否则，删除最远的那一帧
            frame_to_remove = keyframe_.at(max_kf_id);
        }
        LOG(INFO) << "remove keyframe " << frame_to_remove->frame_id_;

        active_keyframe_.erase(frame_to_remove->frame_id_);
        for (auto feat : frame_to_remove->feature_left_) // 遍历这一帧中左侧的特征点
        {
            auto mp = feat->MapPoint_.lock();
            if (mp) // 如果这个特征点有对应的路标点
            {
                mp->removeObservation(feat); // 从路标点中删除这个特征点
            }
        }
        for (auto feat : frame_to_remove->feature_right_)
        {
            if (feat == nullptr)
                continue;
            auto mp = feat->MapPoint_.lock();
            if (mp)
            {
                mp->removeObservation(feat);
            }
        }
        cleanMap(); // 前面删除了路标点中的特征点，因次这里将特征点个数为0的路标点删除
    }

    void Map::cleanMap()
    {
        
        int cnt_landmark_removed = 0;
        for (auto iter = active_landmarks_.begin(); iter != active_landmarks_.end();)
        {
            if (iter->second->observed_times_ == 0)
            {
                iter = active_landmarks_.erase(iter); // 又返回了新的迭代器，因此能够继续遍历
                // active_landmarks_.erase(iter->first); // 错误写法，没有返回新的迭代器，之前的迭代器失效，不能继续遍历
                cnt_landmark_removed++;
            }
            else
            {
                iter++;
            }
        }
        LOG(INFO) << "Removed " << cnt_landmark_removed << " active landmarks";
    }
}