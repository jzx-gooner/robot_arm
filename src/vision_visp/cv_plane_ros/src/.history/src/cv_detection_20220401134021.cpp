//
// Created by jzx
//
#include "cv_detection.hpp"
#include <cv_bridge/cv_bridge.h>
#include "dataman.hpp"
// write file
#include <iostream>
#include <fstream>
// marker
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace cv;



static std::tuple<uint8_t, uint8_t, uint8_t> hsv2bgr(float h, float s, float v)
{
    const int h_i = static_cast<int>(h * 6);
    const float f = h * 6 - h_i;
    const float p = v * (1 - s);
    const float q = v * (1 - f * s);
    const float t = v * (1 - (1 - f) * s);
    float r, g, b;
    switch (h_i)
    {
    case 0:
        r = v;
        g = t;
        b = p;
        break;
    case 1:
        r = q;
        g = v;
        b = p;
        break;
    case 2:
        r = p;
        g = v;
        b = t;
        break;
    case 3:
        r = p;
        g = q;
        b = v;
        break;
    case 4:
        r = t;
        g = p;
        b = v;
        break;
    case 5:
        r = v;
        g = p;
        b = q;
        break;
    default:
        r = 1;
        g = 1;
        b = 1;
        break;
    }
    return make_tuple(static_cast<uint8_t>(b * 255), static_cast<uint8_t>(g * 255), static_cast<uint8_t>(r * 255));
}

static std::tuple<uint8_t, uint8_t, uint8_t> random_color(int id)
{
    float h_plane = ((((unsigned int)id << 2) ^ 0x937151) % 100) / 100.0f;
    ;
    float s_plane = ((((unsigned int)id << 3) ^ 0x315793) % 100) / 100.0f;
    return hsv2bgr(h_plane, s_plane, 1);
}

static bool exists(const string &path)
{

#ifdef _WIN32
    return ::PathFileExistsA(path.c_str());
#else
    return access(path.c_str(), R_OK) == 0;
#endif
}

void CvDetection::init()
{
    ROS_INFO("<< cv detection go!");
    //图像订阅
    img_sub = nh_.subscribe<sensor_msgs::CompressedImage>("/camera/color/image_raw/compressed", 1,
                                                          &CvDetection::imgCallback, this);

    depth_sub = nh_.subscribe("/camera/depth/image_rect_raw", 1, &CvDetection::depthCallback, this);
    depthtoclolor_sub = nh_.subscribe("/camera/extrinsics/depth_to_color", 1, &CvDetection::depth_to_colorCallback, this);
    //相机参数
    subrgbcaminfo = nh_.subscribe("/camera/color/camera_info", 1, &CvDetection::inforgbcallback, this);
    subdepthcaminfo = nh_.subscribe("/camera/depth/camera_info", 1, &CvDetection::infodepthcallback, this);

    //发布
    nh_.param<bool>("is_debug", debug_, true);
    plane_pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/plane/points", 1);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    //服务
    plane_service = nh_.advertiseService("save_plane_result", &CvDetection::saveServerClient, this);

    // 0.load model build yolo class
    printf("TRTVersion: %s\n", SimpleYolo::trt_version());
    int device_id = 0;
    // string model = "yolox_s";
    auto type = SimpleYolo::Type::V5;
    auto mode = SimpleYolo::Mode::FP32;
    string model_path = "/home/jzx/IMPORTANT_MODELS/detection_model.trt";
    SimpleYolo::set_device(device_id);
    float confidence_threshold = 0.50f;
    float nms_threshold = 0.5f;
    yolo_ = SimpleYolo::create_infer(model_path, type, device_id, confidence_threshold, nms_threshold);
    if (yolo_ == nullptr)
    {
        printf("Yolo is nullptr\n");
        return;
    }
    // 1.初始化追踪器
    if (USE_DEEPSORT_)
    {
        auto config = DeepSORT::TrackerConfig();
        config.has_feature = false;
        config.max_age = 1500;
        config.nbuckets = 150;
        config.distance_threshold = 5000.0f;
        config.set_per_frame_motion({0.05, 0.02, 0.1, 0.02,
                                     0.08, 0.02, 0.1, 0.02});
        tracker_ = DeepSORT::create_tracker(config);
    }

    //2.初始化机械臂
    // xarm_c.init(nh_);
	// xarm_c.motionEnable(1);
	// xarm_c.setMode(0);
	// xarm_c.setState(0);
}

void CvDetection::infer(cv::Mat &img)
{
    // auto det_objs = yolo_->commit(img).get();
    // cout << img.size() << endl;
    // cout << "det objets size : " << to_string(det_objs.size()) << std::endl;\

    //基于颜色的基础检测 only for demo
    int hmin = 160, hmax = 180, smin = 43, smax = 255, vmin = 46, vmax = 255;
    int g_nStructElementSize = 3;
    int g_nGaussianBlurValue = 6;

    Mat imghsv;
    cvtColor(img, imghsv, COLOR_BGR2HSV);//RGB to HSV
    // imshow("hsv", imghsv);

    Mat mask;
    inRange(imghsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), mask);//filter red color
    // inRange(mask, Scalar(0, smin, vmin), Scalar(10, smax, vmax), mask);//filter red color
    // imshow("mask", mask);

    Mat out2;
    Mat element = getStructuringElement(MORPH_RECT, Size(2 * g_nStructElementSize + 1, 2 * g_nStructElementSize + 1), Point(g_nStructElementSize, g_nStructElementSize));
    erode(mask, out2, element); //erode
    // imshow("腐蚀", out2);

    Mat gaussian;
    GaussianBlur(out2, gaussian, Size(g_nGaussianBlurValue * 2 + 1, g_nGaussianBlurValue * 2 + 1), 0, 0);//模糊化
    // imshow("高斯滤波", gaussian);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Mat imgcontours;
    Point2f center;
    float radius;


    findContours(gaussian, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    double maxarea = 0;
    int maxareaidx = 0;
    for (int index = contours.size() - 1; index >= 0; index --)// find the maxarea return contour index
    {
    double tmparea = fabs(contourArea(contours[index]));
    if (tmparea > maxarea)
    {
    maxarea = tmparea;
    maxareaidx = index;
    }
    }
    cv::Rect boundingbox = boundingRect(contours[maxareaidx]);
    // minEnclosingCircle(contours[maxareaidx], center, radius);//using index ssearching the min circle
    // circle(img, static_cast<Point>(center), (int)radius, Scalar(255,0,0), 3);//using contour index to drawing circle
    // imshow("轮廓", img);
    cv::rectangle(img, boundingbox, Scalar(0, 0, 255), 2);
    // cv::Rect boundingbox(100,100, 400, 350);
    // cv::rectangle(img, cv::Point(100, 100), cv::Point(500, 450), cv::Scalar(255, 0, 0), -1);

    auto temp = Objection(boundingbox, "ls");

    //发布点云消息
    if(temp.is_plane_finished){
        
        auto plane_pointcloud = temp.cloud_proj;
        sensor_msgs::PointCloud2 plane_pointcloud_msg;
        pcl::toROSMsg(*plane_pointcloud, plane_pointcloud_msg);
        plane_pointcloud_msg.header.frame_id = "world";
        plane_pointcloud_msg.header.stamp = ros::Time::now();
        plane_pointcloud_pub.publish(plane_pointcloud_msg);

        //发布箭头
         visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
    
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = 0;
    
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = visualization_msgs::Marker::ARROW;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;
    
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.points.push_back(temp.center_point) ;
        marker.points.push_back(temp.) ;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 2.0;
        marker.scale.y = 2.0;
        marker.scale.z = 2.0;
    
        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker_pub

        cv::imshow("inference_by_sgai_detection", img);
        cv::waitKey(10);

    }



    //移动机械臂
    // if (SAVE_PLANE_RESULT)
    //     {
    //         std::vector<float> prep_pos = temp.plane_pose;
	//         xarm_c.moveLine(prep_pos, 30, 200);
    //         SAVE_PLANE_RESULT = false;
    //     }
}
void CvDetection::imgCallback(const sensor_msgs::CompressedImage::ConstPtr &image_msg)
{
    try
    {
        if (image_msg->header.seq % 5 == 0) //每隔5帧
        {
            cv::Mat image = cv::imdecode(cv::Mat(image_msg->data), 1); // convert compressed image data to cv::Mat
            dataman::GetInstance()->Setcolormat(image);
            infer(image);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        std::cout << "could not " << std::endl;
    }
}

void CvDetection::inforgbcallback(const sensor_msgs::CameraInfo &caminfo)
{
    if (Colorinfo)
        return;
    else
    {
        ///获取彩色相机内参
        std::cout << "\ncolor intrinsics: " << endl;
        InnerTransformation_Color << caminfo.K.at(0), caminfo.K.at(1), caminfo.K.at(2), caminfo.K.at(3), caminfo.K.at(4), caminfo.K.at(5), caminfo.K.at(6), caminfo.K.at(7), caminfo.K.at(8);
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                std::cout << InnerTransformation_Color(i, j) << "\t";
            }
            std::cout << std::endl;
        }
        dataman::GetInstance()->SetInnerTransformation_Color(InnerTransformation_Color);
        Colorinfo = true;
    }
}
void CvDetection::infodepthcallback(const sensor_msgs::CameraInfo &caminfo)
{
    if (Depthinfo)
        return;
    else
    {
        ///获取深度相机内参
        std::cout << "\ndepth intrinsics: " << endl;
        Inner_Transformation_Depth << caminfo.K.at(0), caminfo.K.at(1), caminfo.K.at(2), caminfo.K.at(3), caminfo.K.at(4), caminfo.K.at(5), caminfo.K.at(6), caminfo.K.at(7), caminfo.K.at(8);
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                std::cout << Inner_Transformation_Depth(i, j) << "\t";
            }
            std::cout << std::endl;
        }
        dataman::GetInstance()->SetInnerTransformation_Depth(Inner_Transformation_Depth);
        Depthinfo = true;
    }
}


void CvDetection::depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    Depthmat = cv_ptr->image;
    dataman::GetInstance()->Setdepthmat(Depthmat);
    this_head = msg->header;
}

void CvDetection::depth_to_colorCallback(const realsense2_camera::Extrinsics &extrin)
{
    if (Convertinfo)
        return;
    else
    {
        ///获取深度相机相对于彩色相机的外参，即变换矩阵: P_color = R * P_depth + T
        std::cout << "\nextrinsics of depth camera to color camera: \nrotaion: " << std::endl;
        MTR << extrin.rotation[0], extrin.rotation[1], extrin.rotation[2], extrin.rotation[3], extrin.rotation[4], extrin.rotation[5], extrin.rotation[6], extrin.rotation[7], extrin.rotation[8];
        dataman::GetInstance()->SetMTR(MTR);
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                std::cout << MTR(i, j) << "\t";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        std::cout << "translation: ";
        V_T << extrin.translation[0], extrin.translation[1], extrin.translation[2];
        dataman::GetInstance()->SetV_T(V_T);
        for (int i = 0; i < 3; i++)
            std::cout << V_T(i) << "\t";
        std::cout << std::endl;
        Convertinfo = true;
    }
}

bool CvDetection::saveServerClient(cv_plane::serverSavePlaneResult::Request &req, cv_plane::serverSavePlaneResult::Response &res)
{
    res.result = "OK";
    SAVE_PLANE_RESULT = true;
    return true;
}