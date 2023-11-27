#ifndef MYSLAM_COMMON_INCLUDE_H
#define MYSLAM_COMMON_INCLUDE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXX;
typedef Eigen::Matrix<double, 3, 3> Mat33;
typedef Eigen::Matrix<double, 3, 4> Mat34;

typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;
typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, 3, 1> Vec3;

#include <sophus/se3.hpp>
#include <sophus/se2.hpp>

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

#include <opencv2/core.hpp>

#include <glog/logging.h>

#endif
