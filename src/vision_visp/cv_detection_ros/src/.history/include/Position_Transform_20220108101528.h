//
// Created by jzx
//
#ifndef DNN_YOLO_POSITION_TRANSFORM_H
#define DNN_YOLO_POSITION_TRANSFORM_H
#include "include.h"
class Position_Transform{
private:
    Eigen::Matrix<float,3,3> CD_Rotation;//相机坐标系旋转矩阵
    Eigen::Vector3f CD_Transation;//平移向量
    Eigen::Matrix<float,3,3> Depth_inner,Color_inner;//相机内参数
    Eigen::Vector3f Depth_cam,Color_cam;
    Eigen::Vector2f RGB_Pix_position;//RGB像素坐标
    std::array<int,2> Depth_Pix;
    std::array<int, 3> PCL_Position ;//RGB相机坐标系坐标
    
    Eigen::Matrix<float,3,3> MTR;//相机坐标旋转矩阵
    Eigen::Vector3f V_T;//平移向量T
    Eigen::Matrix<float,3,3> Inner_Transformation_Depth,InnerTransformation_Color;// 相机内参
    cv::Mat Depthmat, color_mat;
        
public:
    Position_Transform(std::array<int,2> Pix,bool flag);//构造函数 传进rgb或者depth像素坐标
    void Get_camera_referance();//获取相机内参数和转换矩阵
    Eigen::Vector3f Color_PixtoCamera(std::array<int,2> Pix); //彩色图像像素坐标到彩色图像坐标
    Eigen::Vector2f Color_cameraToDepth_Pixel(Eigen::Vector3f Pix_P);//彩色相机坐标到深度像素坐标
    std::array<int,3> Get_XYZ();//返回对应点的相机极坐标系3D位置
    std::array<int,2> Get_Depth_Pixel(std::array<int,2> Pix);
    ~Position_Transform();
};
#endif //DNN_YOLO_POSITION_TRANSFORM_H
