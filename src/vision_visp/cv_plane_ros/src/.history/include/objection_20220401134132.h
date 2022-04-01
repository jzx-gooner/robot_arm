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

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>





using namespace std ;

void EstimateNormal(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
            pcl::PointCloud<pcl::Normal>::Ptr normals);
void FittingPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
            const pcl::PointCloud<pcl::Normal>::Ptr normals,
            pcl::ModelCoefficients::Ptr& coefficients,
            pcl::PointIndices::Ptr& inliers) ;
void visualization(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
            const pcl::PointCloud<pcl::Normal>::Ptr& normals,
            const pcl::ModelCoefficients::Ptr coefficients,
            const pcl::PointIndices::Ptr inliers);    

//typedef dlib::matrix<int ,3,1> sample_type;
//typedef dlib::radial_basis_kernel<sample_type> kernel_type;
class Objection {
    private:
        cv::Rect Aera_Objection_R,Area_Objection_D;
        array<int,2> Point_img;
        vector <array<int ,3>> Objection_PCL;
        vector <array<int,2>> Objection_DepthPoint;
    //    vector<sample_type> samples;//For K-means
    //    vector<sample_type> initial_centers;

        Eigen::Matrix<float,3,3> MTR;//相机坐标旋转矩阵
        Eigen::Vector3f V_T;//平移向量T
        Eigen::Matrix<float,3,3> Inner_Transformation_Depth,InnerTransformation_Color;// 相机内参
        
        cv::Mat Depthmat, color_mat;
    public:
        vector <array<int ,3>> Real_Point;
        array<int ,3> Point_Camera;
        bool Enable;
    //    Mat labels,Stats,centroids;
        string Classname;
        int ClassID;
        Objection(cv::Rect Box,string name);
        void CheckStartPoint();
        void Transform_ImgtoCam();
        void DealRect();//处理对象区域范围
        cv::Rect Area_limit(cv::Rect Box);
        float Get_Area_Depth(cv::Rect Box);
        std::vector<int> center_point;//物体中心+位姿
        std::vector<float> plane_pose;//平面反推点位姿
        std::vector<int> target_point;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proj;
        bool is_plane_finished=true;


};
#endif // OBJECTION_H