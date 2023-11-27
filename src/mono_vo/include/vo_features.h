#ifndef MONO_VO_VO_FEATURES_
#define MONO_VO_VO_FEATURES_

#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <vector>

void featureTracking(cv::Mat img_1, cv::Mat img_2, std::vector<cv::Point2f> &points_1, std::vector<cv::Point2f> &points_2, std::vector<uchar> &status);
void featureDetection(cv::Mat img, std::vector<cv::Point2f> &points);

#endif
