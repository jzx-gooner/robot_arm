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
                cv::line(contoursImage, {width-1, righty}, {0, lefty}, (0, 255, 0), 2);
                cout << lines << endl;

                // RotatedRect rRect = minAreaRect(contours[maxAreaContourId]);
                // float fAngle = rRect.angle;//θ∈（-90度，0]
                // if (fAngle < -45){
                //     fAngle = 90 + fAngle;
                // }
                // std::cout<<"旋转角度是 ： "<<fAngle<<std::endl;
                // Point2f vertices[4];
                // rRect.points(vertices);
                // for (int i = 0; i < 4; i++)
                //     line(contoursImage, vertices[i], vertices[(i+1)%4], Scalar(0,255,0), 2);
                // Rect brect = rRect.boundingRect();
                // rectangle(contoursImage, brect, Scalar(255,0,0), 2);
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
            
            std::vector<float> move_postition{location.center_point[0],location.center_point[1],location.center_point[2]+50,xarm_state[3], xarm_state[4], xarm_state[5]};
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