//
// Created by jzx
//
#ifndef _CV_DETECTION_H_
#define _CV_DETECTION_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/CompressedImage.h"
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include <stdio.h>
#include <cmath>
#include <unistd.h>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <array>
#include <sstream>
#include <random>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdarg.h>
#include <Eigen/Dense>

#include"deepsort.hpp"
#include "simple_yolo.hpp"

#include "realsense2_camera/Extrinsics.h"


#include "ros/ros.h"
#include "include.h"
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "realsense2_camera/Extrinsics.h"
#include <chrono>
#include "time.h"
#include "cv_detection/object.h"
#include "cv_detection/multiobjects.h"

using cv::Mat;

using namespace cv;

#define CV_GREEN cv::Scalar(0, 255, 0)
#define CV_RED cv::Scalar(0, 0, 255)


class MotionFilter
{
    public:
        MotionFilter()
        {
            location_.left = location_.top = location_.right = location_.bottom = 0;
        }

        void missed()
        {
            init_ = false;
        }

        void update(const DeepSORT::Box &box)
        {

            const float a[] = {box.left, box.top, box.right, box.bottom};
            const float b[] = {location_.left, location_.top, location_.right, location_.bottom};

            if (!init_)
            {
                init_ = true;
                location_ = box;
                return;
            }

            float v[4];
            for (int i = 0; i < 4; ++i)
                v[i] = a[i] * 0.6 + b[i] * 0.4;

            location_.left = v[0];
            location_.top = v[1];
            location_.right = v[2];
            location_.bottom = v[3];
        }

        DeepSORT::Box predict()
        {
            return location_;
        }

    private:
        DeepSORT::Box location_;
        bool init_ = false;
    };




class CvDetection {

    public:
        //ros
        ros::NodeHandle nh_;
        std::shared_ptr<DeepSORT::Tracker> tracker_;
        std::shared_ptr<SimpleYolo::Infer> yolo_;
        std::unordered_map<int, MotionFilter> MotionFilter_;
        void init();
        void imgCallback(const sensor_msgs::CompressedImage::ConstPtr &image_msg);
        void infodepthcallback(const sensor_msgs::CameraInfo &caminfo);
        void inforgbcallback(const sensor_msgs::CameraInfo &caminfo);

        void depthCallback(const sensor_msgs::ImageConstPtr& msg);
        void depth_to_colorCallback(const realsense2_camera::Extrinsics &extrin);
        void sendMsgs(sensor_msgs::ImagePtr msg);
        void infer(cv::Mat &img);

    private:
        ros::Subscriber img_sub;
        ros::Subscriber depth_sub;
        ros::Subscriber depthtoclolor_sub;
        ros::Subscriber subrgbcaminfo;
        ros::Subscriber subdepthcaminfo;
        ros::Publisher cvInfo_pub;
        ros::Publisher object_pub;
        ros::Publisher pointcloud_pub;
        bool debug_ = true;
        bool USE_DEEPSORT_=false;
        bool Colorinfo= false;
        bool Depthinfo= false;
        bool Convertinfo = false;

        Eigen::Matrix<float,3,3> MTR;//相机坐标旋转矩阵
        Eigen::Vector3f V_T;//平移向量T
        Eigen::Matrix<float,3,3> Inner_Transformation_Depth,InnerTransformation_Color;// 相机内参
        cv::Mat Depthmat, color_mat;
        std::vector<Objection> objetsin2Dimage;//一幅图中的目标
        std_msgs::Header this_head;
};

inline void displayDot(cv::Mat &img, const cv::Point2i &dotLoc, double dotScale,
                       const cv::Scalar &color = CV_GREEN) {
    cv::circle(img, dotLoc, dotScale / 2.0, color, dotScale / 2.0);
}

inline void
displayText(cv::Mat &img, std::string &text, const cv::Point2i &textLoc, double fontScale, cv::Scalar color) {
    int fontFace = cv::FONT_HERSHEY_PLAIN;//FONT_HERSHEY_COMPLEX_SMALL;//
    int thickness = MAX(fontScale / 2, 2);
    cv::putText(img, text, textLoc, fontFace, fontScale, std::move(color), thickness, 9);
}

static const char *cocolabels[] = {
    "person", "bicycle", "car", "motorcycle", "airplane",
    "bus", "train", "truck", "boat", "traffic light", "fire hydrant",
    "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse",
    "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis",
    "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass",
    "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
    "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush"};




#endif
