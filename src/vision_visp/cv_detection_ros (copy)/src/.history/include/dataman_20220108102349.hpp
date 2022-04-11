// Created by jzx
#include <sys/shm.h>
#include <pthread.h>
#include <semaphore.h>
#include <memory>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
// const int SM_BUFF_SIZE = 10 * 1024 * 1024;

// struct shmstruct {
//     pthread_mutex_t mutex;
//     uint8_t buff[SM_BUFF_SIZE];
// };

#ifndef DATAMAN_HPP
#define DATAMAN_HPP

#ifndef DEG2RAD
#define DEG2RAD                      0.01745329252
#endif

#ifndef RAD2DEG
#define RAD2DEG                      57.295779513
#endif

#ifndef OPENCV_DEP
#define OPENCV_DEP 0
#endif

// template<typename T>
// using Sp = std::shared_ptr<T>;
// template<typename T, typename... Args>
// inline std::shared_ptr<T> makeSp(Args &&... args) {
//     return std::make_shared<T>(std::forward<Args>(args)...);
// }


class dataman {
    public:
        static std::shared_ptr<dataman> GetInstance();

        const int GetData();

        void SetData(const int &blade_brightness_data);

        void SetMTR(Eigen::Matrix<float,3,3> MTR);
        void SetV_T(Eigen::Vector3f V_T);
        void SetInnerTransformation_Depth(Eigen::Matrix<float,3,3> InnerTransformation_Depth);
        void SetInnerTransformation_Color(Eigen::Matrix<float,3,3> InnerTransformation_Color);
        void Setdepthmat(cv::Mat depthmat);
        void Setcolormat(cv::Mat color_mat);

        const Eigen::Matrix<float,3,3> GetMTR();
        const Eigen::Vector3f GetV_T();
        const Eigen::Matrix<float,3,3> GetInnerTransformation_Depth();
        const Eigen::Matrix<float,3,3> GetInnerTransformation_Color();
        const cv::Mat Getdepthmat();
        const cv::Mat Getcolormat();

    private:
        void PrintLastStateData();
    private:
        //mutex m_mutex;
        Eigen::Matrix<float,3,3> MTR_;//相机坐标旋转矩阵
        Eigen::Vector3f V_T_;//平移向量T
        Eigen::Matrix<float,3,3> InnerTransformation_Depth_;// 深度相机内参 
        Eigen::Matrix<float,3,3> InnerTransformation_Color_;// 彩色相机内参
        cv::Mat depth_mat_;
        cv::Mat color_mat_;
        int data_ = 1;
    };
#endif //DATAMAN_HPP
