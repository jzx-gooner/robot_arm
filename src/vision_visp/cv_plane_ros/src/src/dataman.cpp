#include <utility>
#include "dataman.hpp"
#include <cstring>

std::shared_ptr<dataman> dataman::GetInstance() {
    static std::shared_ptr<dataman> l_singleton = nullptr;
    if (!l_singleton) {
        l_singleton = std::make_shared<dataman>();
    }
    return l_singleton;
}

const int dataman::GetData() {
    return data_;
}


void dataman::SetData(const int &blade_brightness_data) {
    {
        // unique_lock<mutex> lk(m_mutex);
        data_ = blade_brightness_data;
    }
}

void dataman::SetMTR(Eigen::Matrix<float,3,3> MTR) {
    {
        // unique_lock<mutex> lk(m_mutex);
        MTR_ = MTR;
    }
}

void dataman::SetV_T(Eigen::Vector3f V_T) {
    {
        // unique_lock<mutex> lk(m_mutex);
        V_T_ = V_T;
    }
}

void dataman::SetInnerTransformation_Depth(Eigen::Matrix<float,3,3> InnerTransformation_Depth) {
    {
        // unique_lock<mutex> lk(m_mutex);
        InnerTransformation_Depth_ = InnerTransformation_Depth;
    }
}

void dataman::SetInnerTransformation_Color(Eigen::Matrix<float,3,3> InnerTransformation_Color) {
    {
        // unique_lock<mutex> lk(m_mutex);
        InnerTransformation_Color_ = InnerTransformation_Color;
    }
}

void dataman::Setdepthmat(cv::Mat depthmat) {
    {
        // unique_lock<mutex> lk(m_mutex);
        depth_mat_ = depthmat;
    }
}

void dataman::Setcolormat(cv::Mat colormat) {
    {
        // unique_lock<mutex> lk(m_mutex);
        color_mat_ = colormat;
    }
}


const Eigen::Matrix<float,3,3> dataman::GetMTR() {
    {
        // unique_lock<mutex> lk(m_mutex);
        return MTR_;
    }
}

const Eigen::Vector3f dataman::GetV_T() {
    {
        // unique_lock<mutex> lk(m_mutex);
        return V_T_;
    }
}



const Eigen::Matrix<float,3,3> dataman::GetInnerTransformation_Depth() {
    {
        // unique_lock<mutex> lk(m_mutex);
        return InnerTransformation_Depth_;
    }
}

const Eigen::Matrix<float,3,3> dataman::GetInnerTransformation_Color() {
    {
        // unique_lock<mutex> lk(m_mutex);
        return InnerTransformation_Color_;
    }
}

const cv::Mat dataman::Getcolormat() {
    {
        // unique_lock<mutex> lk(m_mutex);
        return color_mat_;
    }
}

const cv::Mat dataman::Getdepthmat() {
    {
        // unique_lock<mutex> lk(m_mutex);
        return depth_mat_;
    }
}

