#include "myslam/include/mappoint.h"
#include "myslam/include/feature.h"

namespace myslam
{
    MapPoint::MapPoint(uint16_t id, Eigen::Vector3d &pos)
        : id_(id), pos_(pos)
    {
    }

    void MapPoint::setPos(const Eigen::Vector3d &pos)
    {
        std::unique_lock<std::mutex> pos_lock(pos_mutex_);
        pos_ = pos;
    }
    Eigen::Vector3d MapPoint::getPos()
    {
        std::unique_lock<std::mutex> pos_lock(pos_mutex_);
        return pos_;
    }

    void MapPoint::addObservation(std::shared_ptr<Feature> feature)
    {
        std::unique_lock<std::mutex> pos_lock(pos_mutex_);
        observations_.push_back(feature);
        observed_times_++;
    }

    void MapPoint::removeObservation(std::shared_ptr<Feature> feature)
    {
        std::unique_lock<std::mutex> pos_lock(pos_mutex_);
        observations_.remove(feature);
        feature->MapPoint_.reset();
        observed_times_--;

        // for (auto iter = observations_.begin(); iter != observations_.end(); iter++)
        // {
        //     if (*iter == feature)
        //     {
        //         observations_.erase(iter);
        //         feature->MapPoint_.reset();
        //         observed_times_--;
        //         break; // 如果使用迭代器的方式，erase之后迭代器将会失效，不能继续递增
        //     }
        // }
    }

    std::list<std::shared_ptr<Feature>> MapPoint::getObservation()
    {
        std::unique_lock<std::mutex> pos_lock(pos_mutex_);
        return observations_;
    }

    MapPoint::Ptr MapPoint::createMapPoint()
    {
        static uint64_t MapPoint_id = 0;
        MapPoint::Ptr MapPointPtr(new MapPoint);
        MapPointPtr->id_ = MapPoint_id;
        MapPoint_id++;
        return MapPointPtr;
    }
}