#include "vo_features.h"

using namespace std;
using namespace cv;

void featureTracking(Mat img_1, Mat img_2, vector<Point2f> &points_1, vector<Point2f> &points_2, vector<uchar> &status)
{

    vector<float> err;
    calcOpticalFlowPyrLK(img_1, img_2, points_1, points_2, status, err);

    int indexCorrection = 0;
    for (int i = 0; i < status.size(); i++)
    {
        Point2f pt = points_2.at(i - indexCorrection);
        if (status.at(i) == 0 || pt.x < 0 || pt.y < 0) // 遍历，删除坏值
        {
            status.at(i) = 0;

            points_1.erase(points_1.begin() + i - indexCorrection);
            points_2.erase(points_2.begin() + i - indexCorrection);
            indexCorrection++;
        }
    }
}

void featureDetection(Mat img, vector<Point2f> &points)
{
    vector<KeyPoint> keypoints;
    int fast_threshold = 20;
    bool nonmaxSupperession = true;
    FAST(img,keypoints,fast_threshold,nonmaxSupperession);
    KeyPoint::convert(keypoints,points);
}