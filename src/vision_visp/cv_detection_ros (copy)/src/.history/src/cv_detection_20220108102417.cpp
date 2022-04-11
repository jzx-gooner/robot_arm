//
// Created by jzx
//
#include "cv_detection.hpp"
#include <cv_bridge/cv_bridge.h>
#include "dataman.hpp"
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
    //初始化dataman
    //图像订阅
    img_sub = nh_.subscribe<sensor_msgs::CompressedImage>("/camera/color/image_raw/compressed", 1,
                                                          &CvDetection::imgCallback, this);

    depth_sub = nh_.subscribe("/camera/depth/image_rect_raw", 1, &CvDetection::depthCallback, this);
    depthtoclolor_sub = nh_.subscribe("/camera/extrinsics/depth_to_color", 1, &CvDetection::depth_to_colorCallback, this);
    //相机参数
    subrgbcaminfo = nh_.subscribe("/camera/color/camera_info", 1, &CvDetection::inforgbcallback, this);
    subdepthcaminfo = nh_.subscribe("/camera/depth/camera_info", 1, &CvDetection::infodepthcallback, this);

    //发布
    // cvInfo_pub = nh_.advertise<wa_ros_msgs::CvInfo>("/cv/cv_info", 1);
    object_pub = nh_.advertise<cv_detection::multiobjects>("Objects", 10);
    pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/detection/points", 1);
    nh_.param<bool>("is_debug", debug_, true);

    // 0.load model build yolo class
    printf("TRTVersion: %s\n", SimpleYolo::trt_version());
    int device_id = 0;
    // string model = "yolox_s";
    auto type = SimpleYolo::Type::V5;
    auto mode = SimpleYolo::Mode::FP32;
    string model_path = "/home/jzx/IMPORTANT_MODELS/detection_model.trt";
    SimpleYolo::set_device(device_id);
    float confidence_threshold = 0.4f;
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
}

void CvDetection::infer(cv::Mat &img)
{
    auto det_objs = yolo_->commit(img).get();
    // cout << img.size() << endl;
    // cout << "det objets size : " << to_string(det_objs.size()) << std::endl;
    if (USE_DEEPSORT_)
    {
        vector<DeepSORT::Box> boxes;
        for (int i = 0; i < det_objs.size(); ++i)
        {
            auto &det_obj = det_objs[i];
            std::cout << to_string(det_obj.class_label) << std::endl;
            if (det_obj.class_label == 8)
            { //只有在检测是car的时候才更新追踪
                auto track_box = DeepSORT::convert_to_box(det_obj);
                // track_box.feature = det_obj.feature;
                boxes.emplace_back(std::move(track_box));
            }
        }
        // debug
        //  show_result(img, det_objs);
        tracker_->update(boxes);
        auto final_objects = tracker_->get_objects();
        for (int i = 0; i < final_objects.size(); ++i)
        {
            std::cout << to_string(i) << std::endl;
            auto &obj = final_objects[i];
            auto &filter = MotionFilter_[obj->id()];
            if (obj->time_since_update() == 0 && obj->state() == DeepSORT::State::Confirmed)
            {
                uint8_t b, g, r;
                tie(b, g, r) = random_color(obj->id());
                auto loaction = obj->last_position();
                filter.update(loaction);
                loaction = filter.predict();
                cv::rectangle(img, cv::Point(loaction.left, loaction.top), cv::Point(loaction.right, loaction.bottom), cv::Scalar(b, g, r), 5);
                auto name = cocolabels[0]; // loaction.class_label
                auto caption = cv::format("%s %.2f", name, loaction.confidence);
                auto id = obj->id();
                int width = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;
                cv::rectangle(img, cv::Point(loaction.left - 3, loaction.top - 33), cv::Point(loaction.left + width, loaction.top), cv::Scalar(b, g, r), -1);
                // cv::putText(image, caption, cv::Point(loaction.left, loaction.top - 5), 0, 1, cv::Scalar::all(0), 2, 16);
                cv::putText(img, to_string(id), cv::Point(loaction.left, loaction.top - 5), 0, 1, cv::Scalar::all(0), 2, 16);
            }
            else
            {
                filter.missed();
            }
        }

        cv::imshow("inference_by_yolov5+deepsort", img);
        cv::waitKey(1);
    }
    else
    {
        objetsin2Dimage.clear(); //清空上一幅图像的目标
        // 遍历检测到的目标，可视化，并记录到目标类别
        for (auto &obj : det_objs)
        {
            if (obj.class_label == 67) //手机是67
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
                // 坐标系变换  还是有bug，不知道原因，！！！！！！！！！！！！！！！
                auto temp = Objection(boundingbox, name);
                objetsin2Dimage.push_back(temp);
            }
        }

        cv::imshow("inference_by_yolov5", img);
        cv::waitKey(1);
        // 发布目标 点云信息
        cv_detection::multiobjects objetsin3Dpointcloud;
        cv_detection::object object3d;
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        for (auto obj2d : objetsin2Dimage)
        {   
            pcl::PointXYZRGB p;
            object3d.classname = obj2d.Classname;
            object3d.classID = obj2d.ClassID;
            p.r = 255 - obj2d.ClassID * 3;
            p.g = obj2d.ClassID * 3;
            p.b = obj2d.ClassID * 2;
            object3d.center_point.x = obj2d.Point_Camera.at(0);
            object3d.center_point.y = obj2d.Point_Camera.at(1);
            object3d.center_point.z = obj2d.Point_Camera.at(2);
            cout << "CenterPoint:" << object3d.center_point.x << " " << object3d.center_point.y << " " << object3d.center_point.z << endl;
            for (auto i : obj2d.Real_Point)
            {
                geometry_msgs::Point32 point;
                point.x = i[0] / 1000.0;
                point.y = i[1] / 1000.0;
                point.z = i[2] / 1000.0;
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                object3d.point_cloud.push_back(point);
                cloud.points.push_back(p);
            }
            objetsin3Dpointcloud.multiobjects.push_back(object3d);
        }
        pcl::toROSMsg(cloud, objetsin3Dpointcloud.pointcloud);
        objetsin3Dpointcloud.pointcloud.header.stamp = ros::Time::now();
        objetsin3Dpointcloud.pointcloud.header.frame_id = "objects";
        objetsin3Dpointcloud.sizeofobjections = objetsin2Dimage.size();
        object_pub.publish(objetsin3Dpointcloud);                //发布信息--一幅图像的信息
        pointcloud_pub.publish(objetsin3Dpointcloud.pointcloud); //发布信息--一幅图像点云的信息
        objetsin3Dpointcloud.multiobjects.clear();
    }
}
void CvDetection::imgCallback(const sensor_msgs::CompressedImage::ConstPtr &image_msg)
{
    try
    {
        if (image_msg->header.seq % 1 == 0)
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
        dataman::GetInstance->SetInnerTransformation_Depth(Inner_Transformation_Depth);
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