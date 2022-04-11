//
// Created by jzx
//
#ifndef INCLUDE_H
#define INCLUDE_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <algorithm>
#include <cstdlib>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "Position_Transform.h"
#include <numeric>
#include "objection.h"
#include "cv_detection.hpp"
//#include "mlpack/core.hpp"
#define Stride 5 //稀疏化步长
#define Distance_Limit (Stride*Stride*100)
#define HeightCam (720) //480
#define WidthCam (1280) //848
#define USE_FP16  // comment out this if want to use FP32
#define DEVICE 0  // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1
#define VIDEO_TYPE (3) //0 means laptop camera ;1 means images,2 means Videos,3 means RealsenseD435;
#define NET s  // s m l x
#define NETSTRUCT(str) createEngine_##str
#define CREATENET(net) NETSTRUCT(net)
#define STR1(x) #x
#define STR2(x) STR1(x)
//#define Distance_Limit (Stride*10)

//extern int Stride;
#endif //INCLUDE_H