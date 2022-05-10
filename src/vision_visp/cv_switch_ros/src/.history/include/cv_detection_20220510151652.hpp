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
#include "cv_switch/object.h"
#include "cv_switch/multiobjects.h"
#include "cv_switch/BoundingBox.h"
#include "cv_switch/BoundingBoxes.h"
#include "cv_switch/serverSaveDetectionResult.h"


//x_arm
#include <xarm_msgs/RobotMsg.h>

#include <xarm_api/xarm_ros_client.h>
//yolo
#include "simple_yolo.hpp"

//segmentation
#include "cv_segmentation.hpp"

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
        void detection_infer(cv::Mat img);
        void segmentation_infer(cv::Mat img);
        void xarm_states_callback(const xarm_msgs::RobotMsg::ConstPtr& states);
        bool saveServerClient(cv_switch::serverSaveDetectionResult::Request &req, cv_switch::serverSaveDetectionResult::Response &res);
        void ArmMove(std::vector<float> prep_pos);
        void CvDetection::GriperMove(float angle)
        bool rough_detection();
        bool fine_detection();
        static void mouseHandleStatic( int event, int x, int y, int flags, void* that );
        void onMouse( int event, int x, int y, int flags);
        CvDetection(): nh_("xarm"){ };
    private:
        xarm_api::XArmROSClient xarm_c;
        std::shared_ptr<CvSegmentation> cs;
        ros::Subscriber img_sub;
        ros::Subscriber depth_sub;
        ros::Subscriber depthtoclolor_sub;
        ros::Subscriber subrgbcaminfo;
        ros::Subscriber subdepthcaminfo;
        ros::Subscriber xarm_state_sub;
        ros::Publisher cvInfo_pub;
        ros::Publisher object_pub;
        ros::Publisher switch_pointcloud_pub;
        ros::Publisher detection_pub_2d;
        ros::Publisher markers_pub;
        ros::ServiceServer detection_service;
        bool debug_ = true;
        bool USE_DEEPSORT_=false;
        bool Colorinfo= false;
        bool Depthinfo= false;
        bool Convertinfo = false;
        bool SAVE_DETECTION_RESULT=false;
        bool we_got_something = false;
        bool rect_done_ =false;
        bool draw_box_ = true;


        cv::Rect region_rect_;
        std::vector<double> xarm_state;


        Eigen::Matrix<float,3,3> MTR;//相机坐标旋转矩阵
        Eigen::Vector3f V_T;//平移向量T
        Eigen::Matrix<float,3,3> Inner_Transformation_Depth,InnerTransformation_Color;// 相机内参
        cv::Mat Depthmat, color_mat;
        std::vector<Objection> m_objetsin2Dimage;//一幅图中的目标
        std_msgs::Header this_head;
        typedef enum {
            ST_INIT = 0,
            ST_DETECTION_INFER =1,
            ST_ROUTINE_DETECTION = 2,
            ST_ROUGH_DETECTION = 3,
            ST_FINE_DETECTION = 4,
            ST_SEGMENTATION_INFER = 5,
            ST_COMPLETE = 6
    } ROBOT_ARM_STATE;
    void ProcessState();
    ROBOT_ARM_STATE m_state_ = ST_COMPLETE;
    std::vector<SimpleYolo::Box> det_objs;
    std::vector<float> m_initpostion{506,-35,413,-M_PI, 0, 0};
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
    "lm", "ls", "car", "motorcycle", "airplane",
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
