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

    //机械臂订阅
    xarm_state_sub = nh_.subscribe("/xarm/xarm_states", 1, &CvDetection::xarm_states_callback,this);

    //发布
    object_pub = nh_.advertise<cv_detection::multiobjects>("Objects", 10);
    pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/detection/points", 1);
    detection_pub_2d = nh_.advertise<cv_detection::BoundingBoxes>("/detection/2d_detection", 1);
    markers_pub = nh_.advertise<visualization_msgs::MarkerArray>("/detection/markers", 100);
    nh_.param<bool>("is_debug", debug_, true);

    //服务
    detection_service = nh_.advertiseService("save_detection_result", &CvDetection::saveServerClient, this);

    // 0.load model build yolo class
    printf("TRTVersion: %s\n", SimpleYolo::trt_version());
    int device_id = 0;
    // string model = "yolox_s";
    auto type = SimpleYolo::Type::V5;
    auto mode = SimpleYolo::Mode::FP32;
    string model_path = "/home/jzx/IMPORTANT_MODELS/detection_model.trt";
    SimpleYolo::set_device(device_id);
    float confidence_threshold = 0.65f;
    float nms_threshold = 0.7f;
    yolo_ = SimpleYolo::create_infer(model_path, type, device_id, confidence_threshold, nms_threshold);
    if (yolo_ == nullptr)
    {
        printf("Yolo is nullptr\n");
        return;
    }

}

void CvDetection::infer(cv::Mat &img)
{
    auto det_objs = yolo_->commit(img).get();
    // cout << img.size() << endl;
    // cout << "det objets size : " << to_string(det_objs.size()) << std::endl;

    {
        objetsin2Dimage.clear(); //清空上一幅图像的目标
        cv_detection::BoundingBoxes bboxes;
        visualization_msgs::MarkerArray marker_msg;
        int counter_id = 0;
        // 遍历检测到的目标，可视化，并记录到目标类别
        for (auto &obj : det_objs)
        {
            if (true) //手机是67 obj.class_label == 67 15是猫
            {
                uint8_t b, g, r;
                tie(b, g, r) = random_color(obj.class_label);
                cv::rectangle(img, cv::Point(obj.left, obj.top), cv::Point(obj.right, obj.bottom), cv::Scalar(b, g, r), 5);
                auto name = cocolabels[obj.class_label];
                auto caption = cv::format("%s %.2f", name, obj.confidence);
                int width = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;
                cv::rectangle(img, cv::Point(obj.left - 3, obj.top - 33), cv::Point(obj.left + width, obj.top), cv::Scalar(b, g, r), -1);
                cv::putText(img, caption, cv::Point(obj.left, obj.top - 5), 0, 1, cv::Scalar::all(0), 2, 16);
                cv::Rect boundingbox(int(obj.left), int(obj.top), int(obj.right - obj.left), int(obj.bottom - obj.top));
                auto center = cv::Point(int((obj.left + obj.right) / 2), int((obj.top + obj.bottom) / 2));
                cv::circle(img, center, 5, cv::Scalar(b, g, r), -1);
                // 坐标系变换
                auto temp = Objection(boundingbox, name);
                objetsin2Dimage.push_back(temp);

                //装消息
                cv_detection::BoundingBox bbox;
                bbox.left = obj.left;
                bbox.right = obj.right;
                bbox.top = obj.top;
                bbox.bottom = obj.bottom;
                bbox.class_label = obj.class_label;
                bbox.confidence = obj.confidence;
                bboxes.bounding_boxes.push_back(bbox);

                //可视化装marker
                visualization_msgs::Marker bbx_marker;
                bbx_marker.header.frame_id = "camera_link";
                bbx_marker.header.stamp = ros::Time::now();
                bbx_marker.ns = "cv_detection";
                bbx_marker.id = counter_id++;
                bbx_marker.type = visualization_msgs::Marker::CUBE;
                bbx_marker.action = visualization_msgs::Marker::ADD;
                bbx_marker.pose.position.x = temp.center_point[0];
                bbx_marker.pose.position.y = temp.center_point[1];
                bbx_marker.pose.position.z = temp.center_point[2];
                bbx_marker.pose.orientation.x = 0.0;
                bbx_marker.pose.orientation.y = 0.0;
                bbx_marker.pose.orientation.z = 0.0;
                bbx_marker.pose.orientation.w = 1.0;
                bbx_marker.scale.x = 20;
                bbx_marker.scale.y = 10;
                bbx_marker.scale.z = 2;
                bbx_marker.color.b = 0;
                bbx_marker.color.g = obj.confidence * 255.0;
                bbx_marker.color.r = (1.0 - obj.confidence) * 255.0;
                bbx_marker.color.a = 0.4;
                bbx_marker.lifetime = ros::Duration(0.5);
                marker_msg.markers.push_back(bbx_marker);
            }
        }

        //发布检测到的信息 2d
        bboxes.header.frame_id = "detection_link";
        bboxes.header.stamp = ros::Time::now();
        detection_pub_2d.publish(bboxes);
        cv::imshow("inference_by_sgai_detection", img);
        cv::waitKey(1);

        //发布marker信息
        markers_pub.publish(marker_msg);

    }
}




void CvDetection::imgCallback(const sensor_msgs::CompressedImage::ConstPtr &image_msg)
{
    try
    {
        if (image_msg->header.seq % 3 == 0) //每隔3帧处理一次
        {
            color_mat = cv::imdecode(cv::Mat(image_msg->data), 1); // convert compressed image data to cv::Mat
            dataman::GetInstance()->Setcolormat(color_mat);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        std::cout << "could not " << std::endl;
    }
        //如果服务调用了，启动状态机
        if (SAVE_DETECTION_RESULT)
        {
            m_state_ = ST_INIT;
            SAVE_DETECTION_RESULT = false;
        }

    ProcessState();
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

void CvDetection::sendMsgs(sensor_msgs::ImagePtr msg)
{

    std::cout << "publish the cv result" << std::endl;
    cvInfo_pub.publish(msg);
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

bool CvDetection::saveServerClient(cv_detection::serverSaveDetectionResult::Request &req, cv_detection::serverSaveDetectionResult::Response &res)
{
    res.result = "OK";
    SAVE_DETECTION_RESULT = true;
    return true;
}



void CvDetection::xarm_states_callback(const xarm_msgs::RobotMsg::ConstPtr& states)
{
  printf("[XARM_STATE]\n");
  std::vector<double> temp{states->pose[0],states->pose[1],states->pose[2],states->pose[3],states->pose[4],states->pose[5]};
  dataman::GetInstance()->SetXarmState(temp);
}

//机械臂移动到指定位置
void CvDetection::ArmMove(){
    std::cout<<"move arm"<<std::endl;
}

void CvDetection::ProcessState() {
    switch (m_state_) {
        case ST_INIT: {
            std::cout<<"m_state : ST_INIT"<<std::endl;
            m_result_ = infer(color_mat);
            //推理图片，如果推理结果正确的话，（检测到了物体，概率比较高，nms做的比较好），就进入粗检测， 如果推理结果不正确的话，还在init状态，推理下一帧
            if (m_result_ == RESULT_OK) {
                std::cout<<"m_state : ST_INIT - > ST_ROUGH_DETECTION"<<std::endl;
                m_state_ = ST_ROUGH_DETECTION;
            }
        }
            break;

        case ST_ROUGHT_DETECTION: {
            std::cout<<"m_state : ST_ROUGH_DETECTION"<<std::endl;
            m_result_ = rough_detection(color_mat);
            //输入是图片推理的目标的坐标，计算出目标位置信息，（x,y,z计算的都在安全区域。没有越界），检测成功，进入细检测，如果没有检测成功，返回到init
            if (m_result_ == RESULT_OK) {
                std::cout<<"m_state : ST_ROUGH_DETECTION -> ST_FINE_DETECTION "<<std::endl;
                m_state_ = ST_FINE_DETECTION;
            }else{
                std::cout<<"m_state : ST_ROUGH_DETECTION -> ST_INIT "<<std::endl;
                m_state_ = ST_INIT;
            }
        }
            break;
        case ST_FINE_DETECTION: {
            //输入是目标的位置一组 （x,y,z） 。需要loop执行
            std::cout<<"m_state : ST_FINE_DETECTION "<<std::endl;
            for(auto& location ::locations){
                ArmMove(location);
                m_result_ = fine_infer(color_mat);
                //如果检测成功
                if (m_result_ == RESULT_OK) { 
                    ArmMove(new_location);
                }else{
                    std::cout<<"fine detection failed"<<std::endl;
                }
            }
            //回到初始位置
            m_state_ = ST_COMPLETE;
        }
            break;

        case ST_COMPLETE: {

        }
            break;
        default:
            break;
    }
}