//
// Created by jzx
//
#ifndef OBJECTION_H
#define OBJECTION_H
#include <fstream>
#include <algorithm>
#include <cstdlib>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "dataman.hpp"

// #include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>


using namespace std ;


class Objection {
    private:
        cv::Rect Aera_Objection_R;
        Eigen::Matrix<float,3,3> MTR;//相机坐标旋转矩阵
        Eigen::Vector3f V_T;//平移向量T
        Eigen::Matrix<float,3,3> Inner_Transformation_Depth,InnerTransformation_Color;// 相机内参
        cv::Mat Depthmat, color_mat;
        int detction_mode_ = 0; 
        string Classname;

    public:
        
        Objection(cv::Rect Box, string name);
        Objection(float segmentation_x,float segmentation_y,string name);
        cv::Rect Area_limit(cv::Rect Box);
        std::vector<int> detection_center_point;//检测物体中心+位姿
        std::vector<int> segmentation_center_point;//分割物体中心+位姿
        std::vector<int> camera_in_center_point;//这个是让物体在相机视野中心的位姿
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
        cv::Rect boundingbox;

};
#endif // OBJECTION_H