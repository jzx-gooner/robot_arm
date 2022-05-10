//
// Created by jzx
//

//todo : 1.确定一个初始位置。
//todo : 2.初始位置确定之后，确定routine检测 可不可以

#include "cv_detection.hpp"
#include "cv_segmentation.hpp"
#include <cv_bridge/cv_bridge.h>
#include "dataman.hpp"
// write file
#include <iostream>
#include <fstream>
// marker
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace cv;

#define is_routine_detection false

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


/*定义一些常用函数，有时间在移动重构吧*/
// 判断两条线是否相交，若相交则求出交点和夹角
Vec4d lines_intersection(const Vec4d l1, const Vec4d l2)
{
    double x1 = l1[0], y1 = l1[1], x2 = l1[2], y2 = l1[3];
    double a1 = -(y2 - y1), b1 = x2 - x1, c1 = (y2 - y1) * x1 - (x2 - x1) * y1; // 一般式：a1x+b1y1+c1=0
    double x3 = l2[0], y3 = l2[1], x4 = l2[2], y4 = l2[3];
    double a2 = -(y4 - y3), b2 = x4 - x3, c2 = (y4 - y3) * x3 - (x4 - x3) * y3; // 一般式：a2x+b2y1+c2=0
    bool r = false;                                                             // 判断结果
    double x0 = 0, y0 = 0;                                                      // 交点
    double angle = 0;                                                           // 夹角
    // 判断相交
    if (b1 == 0 && b2 != 0) // l1垂直于x轴，l2倾斜于x轴
        r = true;
    else if (b1 != 0 && b2 == 0) // l1倾斜于x轴，l2垂直于x轴
        r = true;
    else if (b1 != 0 && b2 != 0 && a1 / b1 != a2 / b2)
        r = true;
    if (r)
    {
        //计算交点
        x0 = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1);
        y0 = (a1 * c2 - a2 * c1) / (a2 * b1 - a1 * b2);
        // 计算夹角
        double a = sqrt(pow(x4 - x2, 2) + pow(y4 - y2, 2));
        double b = sqrt(pow(x4 - x0, 2) + pow(y4 - y0, 2));
        double c = sqrt(pow(x2 - x0, 2) + pow(y2 - y0, 2));
        angle = acos((b * b + c * c - a * a) / (2 * b * c)) * 180 / CV_PI;
    }
    return Vec4d(r, x0, y0, angle);
}


// 画夹角
void draw_angle(Mat img, const Point2d p0, const Point2d p1, const Point2d p2, const double radius, const Scalar color, const int thickness)
{
    // 计算直线的角度
    double angle1 = atan2(-(p1.y - p0.y), (p1.x - p0.x)) * 180 / CV_PI;
    double angle2 = atan2(-(p2.y - p0.y), (p2.x - p0.x)) * 180 / CV_PI;
    // 计算主轴的角度
    double angle = angle1 <= 0 ? -angle1 : 360 - angle1;
    // 计算圆弧的结束角度
    double end_angle = (angle2 < angle1) ? (angle1 - angle2) : (360 - (angle2 - angle1));
    if (end_angle > 180)
    {
        angle = angle2 <= 0 ? -angle2 : 360 - angle2;
        end_angle = 360 - end_angle;
    }
    // 画圆弧
    ellipse(img, p0, Size(radius, radius), angle, 0, end_angle, color, thickness);
}

// 画箭头
void draw_arrow(Mat img, const Point2d p1, const Point2d p2, const double angle, const double length, const Scalar color, const int thickness)
{
    double l1 = length * cos(angle * CV_PI / 180), l2 = length * sin(angle * CV_PI / 180);
    Point2d p3(0, 0), p4(0, 0);
    int i = (p2.x > p1.x) ? 1 : -1; // i,j代表p2、p3、p4相对于p0的正负
    int j = (p2.y > p1.y) ? 1 : -1;
    double a1 = abs(atan((p2.y - p1.y) / (p2.x - p1.x))); // 直线p1p2相对于x轴的角度，取正值
    double w1 = l1 * cos(a1), h1 = l1 * sin(a1);          // 用于计算p2相对于p0的宽高
    Point2d p0(p2.x - w1 * i, p2.y - h1 * j);
    double a2 = 90 * CV_PI / 180 - a1;           // 直线p3p4相对于x轴的角度
    double w2 = l2 * cos(a2), h2 = l2 * sin(a2); // 用于计算p3和p4相对于p0的宽高
    p3 = Point2d(p0.x - w2 * i, p0.y + h2 * j);
    p4 = Point2d(p0.x + w2 * i, p0.y - h2 * j);
    line(img, p2, p3, color, 2); //画箭头
    line(img, p2, p4, color, 2);
}

// 画虚线
void draw_dotted_line(Mat img, const Point2d p1, const Point2d p2, const Scalar color, const int thickness)
{
    double n = 15; // 小线段的长度
    double w = p2.x - p1.x, h = p2.y - p1.y;
    double l = sqrtl(w * w + h * h);
    // 矫正小线段长度，使小线段个数为奇数
    int m = l / n;
    m = m % 2 ? m : m + 1;
    n = l / m;

    circle(img, p1, 1, color, thickness); // 画起点
    circle(img, p2, 1, color, thickness); // 画终点
    // 画中间的小线段
    if (p1.y == p2.y) //水平线：y = m
    {
        double x1 = min(p1.x, p2.x);
        double x2 = max(p1.x, p2.x);
        for (double x = x1, n1 = 2 * n; x < x2; x = x + n1)
            line(img, Point2d(x, p1.y), Point2d(x + n, p1.y), color, thickness);
    }
    else if (p1.x == p2.x) //垂直线, x = m
    {
        double y1 = min(p1.y, p2.y);
        double y2 = max(p1.y, p2.y);
        for (double y = y1, n1 = 2 * n; y < y2; y = y + n1)
            line(img, Point2d(p1.x, y), Point2d(p1.x, y + n), color, thickness);
    }
    else
    {
        // 直线方程的两点式：(y-y1)/(y2-y1)=(x-x1)/(x2-x1) -> y = (y2-y1)*(x-x1)/(x2-x1)+y1
        double n1 = n * abs(w) / l;
        double k = h / w;
        double x1 = min(p1.x, p2.x);
        double x2 = max(p1.x, p2.x);
        for (double x = x1, n2 = 2 * n1; x < x2; x = x + n2)
        {
            Point p3 = Point2d(x, k * (x - p1.x) + p1.y);
            Point p4 = Point2d(x + n1, k * (x + n1 - p1.x) + p1.y);
            line(img, p3, p4, color, thickness);
        }
    }
}

// 画延长线
void draw_extension_line(Mat img, const Vec4d l, Scalar color)
{
    double x1 = l[0], y1 = l[1], x2 = l[2], y2 = l[3];
    double a = -(y2 - y1), b = x2 - x1, c = (y2 - y1) * x1 - (x2 - x1) * y1;
    Point2d p1(0, 0), p2(0, 0);
    if (b != 0)
    {
        p1 = Point2d(0, -c / b);
        p2 = Point2d(img.cols, ((-a * img.cols - c) / b));
    }
    else
    {
        p1 = Point2d(-c / a, 0);
        p2 = Point2d(-c / a, img.rows);
    }
    draw_dotted_line(img, p1, p2, color, 1);
}

// 画图
void draw(Mat img, const Vec4d l1, const Vec4d l2)
{
    Vec4d v = lines_intersection(l1, l2); // 判断是否相交，并计算交点和夹角

    line(img, Point2d(l1[0], l1[1]), Point2d(l1[2], l1[3]), Scalar(255, 0, 0), 2);               // 画蓝线
    draw_arrow(img, Point2d(l1[0], l1[1]), Point2d(l1[2], l1[3]), 20, 20, Scalar(255, 0, 0), 2); // 画箭头

    line(img, Point2d(l2[0], l2[1]), Point2d(l2[2], l2[3]), Scalar(0, 255, 0), 2);               // 画绿线
    draw_arrow(img, Point2d(l2[0], l2[1]), Point2d(l2[2], l2[3]), 20, 20, Scalar(0, 255, 0), 2); // 画箭头

    draw_extension_line(img, l1, Scalar(255, 0, 0)); // 画延长线
    draw_extension_line(img, l2, Scalar(0, 255, 0)); // 画延长线

    draw_angle(img, Point2d(v[1], v[2]), Point2d(l1[2], l1[3]), Point2d(l2[2], l2[3]), 15, Scalar(0, 0, 255), 1); // 画夹角

    if (v[0])
    {
        string s = "(" + to_string(v[1]) + ", " + to_string(v[2]) + ") " + to_string(v[3]);
        putText(img, s, Point2d(10, 25), cv::FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 0));
        circle(img, Point2d(v[1], v[2]), 2, Scalar(0, 0, 255), 2); // 画交点
    }
    else
    {
        putText(img, "no", Point2d(10, 25), cv::FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 0, 255));
    }
    cv::imshow("img", img);
    cv::waitKey(1);
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
    object_pub = nh_.advertise<cv_switch::multiobjects>("Objects", 10);
    switch_pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/detection_switch/points", 1);
    detection_pub_2d = nh_.advertise<cv_switch::BoundingBoxes>("/detection/2d_detection", 1);
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
    string model_path = "/home/jzx/robot_arm_models/yolov5s.engine";
    SimpleYolo::set_device(device_id);
    float confidence_threshold = 0.25f;
    float nms_threshold = 0.5f;
    yolo_ = SimpleYolo::create_infer(model_path, type, device_id, confidence_threshold, nms_threshold);
    if (yolo_ == nullptr)
    {
        printf("Yolo is nullptr\n");
        return;
    }


    //1.load segmentation model
    ROS_INFO("<< add segmentation model!");
    std::string segmentaion_model_file = "/home/jzx/robot_arm_models/unet.engine";
    cs = std::make_shared<CvSegmentation>();
    cs->getEngine(segmentaion_model_file);

    //2.初始化机械臂
    xarm_c.init(nh_);
	xarm_c.motionEnable(1);
	xarm_c.setMode(0);
	xarm_c.setState(0);

}

void CvDetection::detection_infer(cv::Mat img)
{
    m_objetsin2Dimage.clear(); //清空上一幅图像的目标
    auto detection_image = img.clone();
    det_objs = yolo_->commit(detection_image).get();
    // cout << "det objets size : " << to_string(det_objs.size()) << std::endl;
    cv::Mat show_img;
    cv::namedWindow("detection_result");
    // cv::resize(color_mat,show_img,cv::Size(800,640));

    if (det_objs.empty() or det_objs.empty()>1 )
    {
        we_got_something = false;
        // cv::resize(img,show_img,cv::Size(800,640));
        cv::imshow("detection_result",detection_image);
        cv::waitKey(1);
    }else{
        for (auto &obj : det_objs)
        {
            if (true) //手机是67 obj.class_label == 67 15是猫
            {
                uint8_t b, g, r;
                tie(b, g, r) = random_color(obj.class_label);
                cv::rectangle(detection_image, cv::Point(obj.left, obj.top), cv::Point(obj.right, obj.bottom), cv::Scalar(b, g, r), 5);
                auto name = cocolabels[obj.class_label];
                auto caption = cv::format("%s %.2f", name, obj.confidence);
                int width = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;
                cv::rectangle(detection_image, cv::Point(obj.left - 3, obj.top - 33), cv::Point(obj.left + width, obj.top), cv::Scalar(b, g, r), -1);
                cv::putText(detection_image, caption, cv::Point(obj.left, obj.top - 5), 0, 1, cv::Scalar::all(0), 2, 16);
                cv::Rect boundingbox(int(obj.left), int(obj.top), int(obj.right - obj.left), int(obj.bottom - obj.top));
                auto center = cv::Point(int((obj.left + obj.right) / 2), int((obj.top + obj.bottom) / 2));
                cv::circle(detection_image, center, 5, cv::Scalar(b, g, r), -1);
                // 坐标系变换 粗检测的坐标系变换
                auto temp = Objection(boundingbox, 0,0,0,name);
                m_objetsin2Dimage.push_back(temp);
            }
        }
            //获取物体的点云
        // for(auto& location : m_objetsin2Dimage){
        //         auto switch_pointcloud = location.raw_cloud;
        //         sensor_msgs::PointCloud2 switch_pointcloud_msg;
        //         pcl::toROSMsg(*switch_pointcloud, switch_pointcloud_msg);
        //         switch_pointcloud_msg.header.frame_id = "world";
        //         switch_pointcloud_msg.header.stamp = ros::Time::now();
        //         switch_pointcloud_pub.publish(switch_pointcloud_msg);
        // }


        // cv::resize(color_mat,show_img,cv::Size(800,640));
        cv::imshow("detection_result",detection_image);
        cv::waitKey(1);
        we_got_something = true;
    }

}


void CvDetection::segmentation_infer(cv::Mat img){
        auto segmentation_image = img.clone();
        cv::imshow("segmentation_image",segmentation_image);
        auto segmentation_img = cs->inference(segmentation_image);
        cv::imshow("segmentation_result",segmentation_img);
        cv::Mat binary_image,gray_image;
        cvtColor(segmentation_img, gray_image, CV_BGR2GRAY);
        cv::adaptiveThreshold(gray_image, binary_image, 255, CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY_INV, 25, 10); ///局部自适应二值化函数
        //去噪
        Mat de_noise = binary_image.clone();
        //中值滤波
        medianBlur(binary_image, de_noise, 5);
        ///////////////////////// 膨胀 ////////////////////
        Mat dilate_img;
        Mat element = getStructuringElement(MORPH_RECT, Size(20, 20/*15, 15*/));
        dilate(de_noise, dilate_img,element);
       //检测连通域，每一个连通域以一系列的点表示，FindContours方法只能得到第一个域
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(binary_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//CV_RETR_EXTERNAL只检测外部轮廓，可根据自身需求进行调整
        
        Mat contoursImage(dilate_img.rows, dilate_img.cols, CV_8U, Scalar(255));
        
        //找最大连通区域
        double maxArea = 0;
        int maxAreaContourId = -1;
        for (int j = 0; j < contours.size(); j++) {
            double newArea = cv::contourArea(contours.at(j));
            if (newArea > maxArea) {
                maxArea = newArea;
                maxAreaContourId = j;
            } // End if
        } // End for
        if(maxAreaContourId>-1){
                cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
                cv::drawContours(contoursImage, contours, maxAreaContourId, Scalar(0), 1, 8, hierarchy);//描绘字符的外轮廓
                Rect rect = boundingRect(contours[maxAreaContourId]);
                rectangle(contoursImage,rect,Scalar(255,0,0), 2);
                Vec4f lines;

                double param = 0.0;
                double reps = 0.01;
                double aeps = 0.01;
                fitLine(contours[maxAreaContourId], lines, DIST_L1, param, reps, aeps);
                Point2d p0 = {lines[2],lines[3]};
                float slope = -lines[1]/lines[0];
                int width = img.size().width;
                int height = img.size().height;
                int lefty = int((lines[2]*slope) + lines[3]);
                int righty = int(((lines[2]-width)*slope)+lines[3]);
                cv::line(segmentation_image, {width-1, righty}, {0, lefty}, (255, 255, 0), 2);
                cout << lines << endl;
                vec4f line1(lines[0],-lines[1],lines(2),lines(3));
                Vec4f line2(0,height/2,width/2,height/2);
                draw(segmentation_image, line1,line2);
        }

        
        // cv::Rect boundingRect = rotatedRect.boundingRect();//返回包含旋转矩形的最小矩形
        // cv::Mat newSrc = srcCopy(boundingRect);
        // //ROI区域倾斜矩形的外接矩形，的旋转中心.
        // cv::Point newCenter;
        // newCenter.x = rotatedRect.center.x - boundingRect.x;
        // newCenter.y = rotatedRect.center.y - boundingRect.y;


        cv::imshow("switch-pose",contoursImage);
        cv::waitKey(1);

}

bool CvDetection::fine_detection(){

    return false;
}

void CvDetection::imgCallback(const sensor_msgs::CompressedImage::ConstPtr &image_msg)
{
    try
    {
        if (image_msg->header.seq % 3 == 0) //每隔3帧处理一次
        {
            color_mat = cv::imdecode(cv::Mat(image_msg->data), 1); // convert compressed image data to cv::Mat
            dataman::GetInstance()->Setcolormat(color_mat);
            detection_infer(color_mat);
            segmentation_infer(color_mat);
            
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

bool CvDetection::saveServerClient(cv_switch::serverSaveDetectionResult::Request &req, cv_switch::serverSaveDetectionResult::Response &res)
{
    res.result = "OK";
    SAVE_DETECTION_RESULT = true;
    return true;
}


 void CvDetection::mouseHandleStatic(int event, int x, int y, int flags, void* param )
 {
     CvDetection* thiz = static_cast<CvDetection*>(param);
     thiz->onMouse(event, x, y, flags);
 }


void CvDetection::onMouse( int event, int x, int y, int flags)
{
    if (rect_done_)
      return;
    switch (event)
    {
    case cv::EVENT_MOUSEMOVE:
    {
      region_rect_.width = x - region_rect_.x;
      region_rect_.height = y - region_rect_.y;
    }
    break;
    case cv::EVENT_LBUTTONDOWN:
    {
      region_rect_ = cv::Rect(x, y, 0, 0);
      draw_box_ = true;
    }
    break;
    case cv::EVENT_LBUTTONUP:
    {
      draw_box_ = false;
      if (region_rect_.width < 0)
      {
        region_rect_.x += region_rect_.width;
        region_rect_.width *= -1;
      }
      if (region_rect_.height < 0)
      {
        region_rect_.y += region_rect_.height;
        region_rect_.height *= -1;
      }
      rect_done_ = true;
    }
    }
}


void CvDetection::xarm_states_callback(const xarm_msgs::RobotMsg::ConstPtr& states)
{
  xarm_state = {states->pose[0],states->pose[1],states->pose[2],states->pose[3],states->pose[4],states->pose[5]};
  dataman::GetInstance()->SetXarmState(xarm_state);
}

//机械臂移动到指定位置
void CvDetection::ArmMove(std::vector<float> prep_pos){
    std::cout<<"move arm : "<< prep_pos[0]<<","<<prep_pos[1]<<","<<prep_pos[2]<<std::endl;
    xarm_c.moveLine(prep_pos, 30, 200);
}


void CvDetection::ProcessState() {
    switch (m_state_) {
        case ST_INIT: {
            std::cout<<"m_state : ST_INIT"<<std::endl;
            //移动到初始位置  --tod0 确定初始位置
            ArmMove(m_initpostion);
            std::cout<<"m_state : ST_INIT - > ST_DETECTION_INFER"<<std::endl;
            //进入infer状态
            m_state_ = ST_DETECTION_INFER;
        }
            break;

        case ST_DETECTION_INFER: {
            std::cout<<"m_state : ST_DETECTION_INFER"<<std::endl;
            //推理图片，如果推理结果正确的话，m_objetsin2Dimage 里面已经更新了信息了，粗检测直接拿去用了，精检测先靠近一下，在拿去用，精简测的话，需要更新一下点
            //沿着当前点与目标点的法向量，延伸10cm！
            if (we_got_something) {
                if(is_routine_detection){
                    std::cout<<"m_state : ST_DETECTION_INFER -> ST_ROUTINE_DETECTION"<<std::endl;
                    m_state_ = ST_ROUTINE_DETECTION;
                }else{
                    m_state_ = ST_FINE_DETECTION;
                }
            }else{
                detection_infer(color_mat);
            }
        }
            break;

        case ST_ROUTINE_DETECTION: {
            std::cout<<"m_state : ST_ROUTINE_DETECTION"<<std::endl;
            //推理图片，如果推理结果正确的话，（检测到了物体，概率比较高，nms做的比较好），就进入粗检测， 如果推理结果不正确的话，还在init状态，推理下一帧
    
            for(auto& location : m_objetsin2Dimage){

       
                //获取物体的点云
                std::cout<<"控制机械臂"<<std::endl;
                //得到这个点云的中心点，计算出点和最后一个机械臂的rotation

                //到达目标位置
                std::vector<float> move_postition{location.center_point[0],location.center_point[1],location.center_point[2],M_PI, 0, 0};
                ArmMove(move_postition);
                //返回初始位置
                ArmMove(m_initpostion);
                //如果检测成功
            }
            std::cout<<"m_state : ST_INIT - > ST_COMPLETE"<<std::endl;
            m_state_ = ST_COMPLETE;
            }
            break;

        case ST_FINE_DETECTION: {
            //输入是目标的位置一组 （x,y,z） 。需要loop执行
            std::cout<<"m_state : ST_FINE_DETECTION "<<std::endl;
            //根据服务的消息，确定处理哪一个，所以不loop了
            int index = 0;
            auto location = m_objetsin2Dimage[index];
            
            // //0.算出来的点
            // Eigen::Vector4d input(location.center_point[0], location.center_point[1], location.center_point[2],1);
            // //1.输出的点
            // Eigen::Vector4d output;
            // //2.转换矩阵
            // double qw = 0.7013088518485089;
            // double qx = 0.0039751934245023735;
            // double qy = -0.003477682492098677;
            // double qz = 0.7128379885223908;
            // double tx = 69.1845508606165;
            // double ty = -30.68690881661964;
            // double tz = -188.596799;
            // //旋转矩阵 初始化顺序，wxyz
            // Eigen::Quaterniond q(qw,qx,qy,qz);
            // q.normalize();
            // Eigen::Matrix3d R = q.toRotationMatrix();
            // //平移矩阵
            // Eigen::Vector3d T = Eigen::Vector3d(tx,ty,tz);
            // //相机坐标系到工具坐标系的变换矩阵
            // Eigen::Matrix4d Trans_ObjToTool;
            // Trans_ObjToTool.setIdentity();
            // Trans_ObjToTool.block<3,3>(0,0) = R;
            // Trans_ObjToTool.block<3,1>(0,3) = T;
            // //计算新点
            // output = input*Trans_ObjToTool.inverse();


            std::vector<float> move_postition{location.center_point[0], location.center_point[1], location.center_point[2]+100,xarm_state[3], xarm_state[4], xarm_state[5]};
            ArmMove(move_postition);
            m_state_ = ST_SEGMENTATION_INFER;
        }
            break;

        case ST_SEGMENTATION_INFER: {
            std::cout<<"m_state : ST_SEGMENTATION_INFER "<<std::endl;
            //创建一个全局变量，计算当前的roll 和  中心点 得到这个位置
        }
            break;

        case ST_COMPLETE: {
        }
            break;
        default:
            break;
    }
}