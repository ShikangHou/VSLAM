#include "vo_features.h"
#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

// double getAbsoluteScale(int fream_id)
// {
//     ifstream path_file("/home/hsk/Documents/visual_slam/mono_vo/KITTI/01/01.txt");
//     string line;
//     double x_prev, y_prev, z_prev, x, y, z;
//     for (int i = 1; i <= fream_id; i++)
//     {
//         getline(path_file, line);
//         if (i == fream_id - 1)
//         {
//             std::istringstream in(line);
//             for (int j = 0; j < 12; j++)
//             {
//                 in >> z_prev;
//                 if (j == 3)
//                     x_prev = z_prev;
//                 else if (j == 7)
//                     y_prev = z_prev;
//             }
//         }
//         if (i == fream_id)
//         {
//             std::istringstream in(line);
//             for (int j = 0; j < 12; j++)
//             {
//                 in >> z;
//                 if (j == 3)
//                     x = z;
//                 else if (j == 7)
//                     y = z;
//             }
//         }
//     }
//     path_file.close();
//     return (x - x_prev) * (x - x_prev) + (y - y_prev) * (y - y_prev) + (z - z_prev) * (z - z_prev);
// }

double getAbsoluteScale(int frame_id)
{

    string line;
    int i = 0;
    ifstream myfile("/home/hsk/Documents/visual_slam/VO/KITTI/01/01.txt");
    double x = 0, y = 0, z = 0;
    double x_prev, y_prev, z_prev;
    if (myfile.is_open())
    {
        while ((getline(myfile, line)) && (i <= frame_id))
        {
            z_prev = z;
            x_prev = x;
            y_prev = y;
            std::istringstream in(line);
            // cout << line << '\n';
            for (int j = 0; j < 12; j++)
            {
                in >> z;
                if (j == 7)
                    y = z;
                if (j == 3)
                    x = z;
            }

            i++;
        }
        myfile.close();
    }

    else
    {
        cout << "Unable to open file";
        return 0;
    }

    return sqrt((x - x_prev) * (x - x_prev) + (y - y_prev) * (y - y_prev) + (z - z_prev) * (z - z_prev));
}

int main(int argc, char const *argv[])
{
    Mat R_f,t_f;
    Mat img_1, img_2;
    vector<Point2f> points_1, points_2;
    int fream_id = 0;
    char image_path[100];

    sprintf(image_path, "/home/hsk/Documents/visual_slam/VO/KITTI/01/image_0/%06d.png", 0);
    Mat img_1_c = imread(image_path);
    cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
    sprintf(image_path, "/home/hsk/Documents/visual_slam/VO/KITTI/01/image_0/%06d.png", 1);
    Mat img_2_c = imread(image_path);
    cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

    if (!img_1.data || !img_2.data)
    {
        cout << " Error reading images" << endl;
        return -1;
    }

    featureDetection(img_1, points_1);
    vector<uchar> status;
    featureTracking(img_1, img_2, points_1, points_2, status);

    double focal = 718.8560;
    cv::Point2d pp(607.1928, 185.2157);

    Mat E, R, t;
    E = findEssentialMat(points_2, points_1, focal, pp, RANSAC, 0.999, 1.0);
    recoverPose(E, points_2, points_1, R, t, focal, pp);

    Mat img_previous = img_2;
    vector<Point2f> points_previous = points_2;
    Mat img_current_c, img_current;
    vector<Point2f> points_current;
    R_f = R.clone();
    t_f = t.clone();

    Mat traj = Mat::zeros(600, 600, CV_8UC3);

    char text[100];
    cv::Point text_org(10, 50);

    for (fream_id = 2; fream_id < MAX_FRAME; fream_id++)
    {
        sprintf(image_path, "/home/hsk/Documents/visual_slam/VO/KITTI/01/image_0/%06d.png", fream_id);
        img_current_c = imread(image_path);
        cvtColor(img_current_c, img_current, COLOR_BGR2GRAY);
        vector<uchar> status;
        featureTracking(img_previous, img_current, points_previous, points_current, status);

        if (points_previous.size() < MIN_NUM_FEAT)
        {
            featureDetection(img_previous, points_previous);
            featureTracking(img_previous, img_current, points_previous, points_current, status);
        }

        E = findEssentialMat(points_current, points_previous, focal, pp, RANSAC, 0.999, 1.0);
        recoverPose(E, points_current, points_previous, R, t, focal, pp);

        double scale = getAbsoluteScale(fream_id);

        if ((scale > 0.1))
        {
            t_f = t_f + scale * (R_f * t);
            R_f = R * R_f;
        }
        else
        {
            cout << "scale below 0.1,or incorrect translation" << endl;
        }

        img_previous = img_current.clone();
        points_previous = points_current;

        circle(traj, Point(t_f.at<double>(0) + 100, 500 + t_f.at<double>(2)), 1, Scalar(0, 0, 255), 2);

        rectangle(traj, Point(10, 30), Point(550, 50), Scalar(0, 0, 0), FILLED);
        sprintf(text, "coordinates: x = %02f m, y = %02f m, z = %02f m", t_f.at<float>(0), t_f.at<float>(1), t_f.at<float>(2));
        putText(traj, text, text_org, 1, 1, Scalar(255, 255, 255));

        imshow("road image", img_current_c);
        imshow("trajectory", traj);

        waitKey(1);
    }

    return 0;
}
