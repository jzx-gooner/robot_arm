//
// Created by jzx
//
#include "objection.h"
#include "Algorithm_Objection_3D.h"
#include "time.h"
#include "include.h"


using namespace std;


Objection::Objection(cv::Rect Box, float roll,float segmentation_center_x ,float segmentation_center_y,string name){
    //1.获取数据
    MTR = dataman::GetInstance()->GetMTR();
    V_T = dataman::GetInstance()->GetV_T();
    Inner_Transformation_Depth = dataman::GetInstance()->GetInnerTransformation_Depth();
    InnerTransformation_Color = dataman::GetInstance()->GetInnerTransformation_Color();
    Depthmat = dataman::GetInstance()->Getdepthmat();
    color_mat = dataman::GetInstance()->Getcolormat();
    Classname=name;//目标类别名称
    // cout<<"Depthmat.size:"<<Depthmat.size()<<endl;
    //2.初始化RGB图像目标框
    Aera_Objection_R=Area_limit(Box);
    //3.获得目标中心点的坐标
    int center_x = (Aera_Objection_R.x+Aera_Objection_R.width/2);
    int center_y = (Aera_Objection_R.y+Aera_Objection_R.height/2);
    // cout<<"center_x:"<<center_x<<"center_y:"<<center_y<<endl;
    //4.计算三维坐标
    //接下来这个类，有点没写好，也有点懒得改了
    //先把图像坐标转换为相机坐标
    Position_Transform PT(array<int,2>{center_x,center_y}, true);
    std::array<int, 3> center_location=PT.Get_XYZ();//转换
    ostringstream center_ss;
    center_ss << "("<<static_cast<int>(center_location[0])<<","<<static_cast<int>(center_location[1])<<","<<static_cast<int>(center_location[2]) <<")";
    //先求一个能把物体放到视野中心的位置
    Eigen::Vector3f camera_in_center_world = PT.Get_CAMERA_TOOL_XYZ();
    camera_in_center_point.push_back(camera_in_center_world[0]);
    camera_in_center_point.push_back(camera_in_center_world[1]);
    camera_in_center_point.push_back(camera_in_center_world[2]);
    //再把相机坐标转换到机械臂坐标
    Eigen::Vector3f grasp_world = PT.Get_ROBOT_TOOL_XYZ();
    ostringstream grasp_ss;
    grasp_ss<<""<<static_cast<int>(grasp_world[0])<<","<<static_cast<int>(grasp_world[1])<<","<<static_cast<int>(grasp_world[2])<<"";
    center_point.push_back(grasp_world[0]);
    center_point.push_back(grasp_world[1]);
    center_point.push_back(grasp_world[2]);
    center_point.push_back(0);
    putText(color_mat, grasp_ss.str(), Point(center_x, center_y-60), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 1);
    //5.获得该框内的所有像素点
    std::cout << "图像平面中心点: " << grasp_world[0] << ", " << grasp_world[1] << ", " << grasp_world[2] <<  std::endl;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // int w = Aera_Objection_R.width;
    // int h = Aera_Objection_R.height;
    // int x_start = Aera_Objection_R.x;
    // int y_start = Aera_Objection_R.y;
    // //稀疏化取点
    // cout<<"w "<<w <<"h "<<h<<"x_start "<<x_start<<"y_start "<<y_start<<endl;
    // int stride = 1;
    // for(int i= x_start; i<x_start+w-stride; i+=stride){
    //     for(int j=y_start; j<y_start+h-stride; j+=stride){
    //         pcl::PointXYZ CurrentPoint;
    //         Position_Transform PT(array<int,2>{i,j},true);
    //         std::array<int, 3> center_location=PT.Get_XYZ();//转换


    //         Eigen::Vector3f grasp_world = PT.Get_ROBOT_TOOL_XYZ();
    //         CurrentPoint ={grasp_world[0],grasp_world[1],grasp_world[2]};

    //         if(grasp_world[0]==0 or grasp_world[1]==0 or grasp_world[2]==0){
    //             continue;
    //         }
    //         // std::cout<<"real point  x,y,z: "<<grasp_world[0]<<" , "<<grasp_world[1]<<" , "<<grasp_world[2]<<std::endl;
    //         temp_cloud->points.push_back(CurrentPoint);
    //     }
    // }

    // std::cout<<"raw_cloud->points.size()"<<temp_cloud->points.size()<<std::endl;

    // // variance(raw_cloud);

    // raw_cloud = temp_cloud;
    


}
cv::Rect  Objection::Area_limit(cv::Rect Box) {
    cv::Rect Obj;
    Obj.x=(Box.x<0 ? 0:Box.x);//起始点越界检测
    Obj.y=(Box.y<0 ? 0:Box.y);
    Obj.height=(Box.y<0 ? Box.height+Box.y:Box.height); //起始点越界修正目标的宽度和高度
    Obj.width=(Box.x<0 ? Box.width+Box.x:Box.width);
    Obj.height=(Obj.height+Obj.y>(HeightCam-1) ? (HeightCam-1)-Obj.y:Obj.height);//目标框大小越界检测
    Obj.width=(Obj.width+Obj.x>(WidthCam-1) ? (WidthCam-1)-Obj.x:Obj.width);
    return Obj;
}
