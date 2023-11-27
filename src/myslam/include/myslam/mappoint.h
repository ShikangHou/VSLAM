#ifndef MYSLAM_MapPoint_H
#define MYSLAM_MapPoint_H

#include "myslam/common_include.h"
#include <mutex>
#include <list>

namespace myslam
{
    class Frame;
    class Feature;

    class MapPoint
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<MapPoint> Ptr;

        uint64_t id_ = 0; // 路标点id
        bool is_outlier_ = false; 
        Eigen::Vector3d pos_ = Eigen::Vector3d::Zero();
        std::mutex pos_mutex_;
        uint64_t observed_times_ = 0;
        std::list<std::shared_ptr<Feature>> observations_; // 该路标点对应的特征点

    public:
        MapPoint(){};
        MapPoint(uint16_t id, Eigen::Vector3d &pos);

        void setPos(const Eigen::Vector3d &pos);
        Eigen::Vector3d getPos();

        void addObservation(std::shared_ptr<Feature> feature);
        void removeObservation(std::shared_ptr<Feature> feature);
        std::list<std::shared_ptr<Feature>> getObservation();

        static Ptr createMapPoint();
    };

}

#endif
