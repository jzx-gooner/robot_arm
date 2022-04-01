//
// Created by jzx
//
#include "objection.h"
#include "Algorithm_Objection_3D.h"
#include "time.h"
#include "include.h"
using namespace std;


Objection::Objection(cv::Rect Box, string name){
    //1.获取数据
    MTR = dataman::GetInstance()->GetMTR();
    V_T = dataman::GetInstance()->GetV_T();
    Inner_Transformation_Depth = dataman::GetInstance()->GetInnerTransformation_Depth();
    InnerTransformation_Color = dataman::GetInstance()->GetInnerTransformation_Color();
    Depthmat = dataman::GetInstance()->Getdepthmat();
    color_mat = dataman::GetInstance()->Getcolormat();
    Classname=name;//目标类别名称
    cout<<"Depthmat.size:"<<Depthmat.size()<<endl;
    //2.初始化RGB图像目标框
    Aera_Objection_R=Area_limit(Box);
    //3.获得目标中心点的坐标
    int center_x = (Aera_Objection_R.x+Aera_Objection_R.width/2);
    int center_y = (Aera_Objection_R.y+Aera_Objection_R.height/2);
    cout<<"center_x:"<<center_x<<"center_y:"<<center_y<<endl;
    Position_Transform PT(array<int,2>{center_x,center_y}, true);
    std::array<int, 3> center_location=PT.Get_XYZ();//转换
    ostringstream center_ss;
    center_ss << "("<<static_cast<int>(center_location[0])<<","<<static_cast<int>(center_location[1])<<","<<static_cast<int>(center_location[2]) <<")";
    Eigen::Vector3f grasp_world = PT.Get_ROBOT_TOOL_XYZ();
    ostringstream grasp_ss;
    grasp_ss<<""<<static_cast<int>(grasp_world[0])<<","<<static_cast<int>(grasp_world[1])<<","<<static_cast<int>(grasp_world[2])<<"";
    putText(color_mat, grasp_ss.str(), Point(center_x, center_y-20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 1);
    putText(color_mat, center_ss.str(), cv::Point(center_x, center_y), 0, 1, cv::Scalar(0, 255, 0));
    //3.转换彩色图的坐标到深度图,得到深度图的目标框
    // Position_Transform Start_point_D(array<int,2>{Aera_Objection_R.x,Aera_Objection_R.y}, true);
    // array<int,2> Start_pix=Start_point_D.Get_Depth_Pixel(array<int,2>{Aera_Objection_R.x,Aera_Objection_R.y});//转换
    // Box.x=Start_pix.at(0);Box.y=Start_pix.at(1);//更新矩形框
    // Area_Objection_D=Area_limit(Box);//越界限制下初始化深度图下的矩形区域

    //4.处理矩形框,稀疏化,获取中心点,聚类,获取该类别物体的点云
    // DealRect();
    //5.
    // CheckStartPoint();
    // Transform_ImgtoCam();//将中心点坐标转换
    // ostringstream ss;
    // if (Enable==true)
    //     ss << "("<<static_cast<int>(Point_Camera.at(0))<<","<<static_cast<int>(Point_Camera.at(1))<<","<<static_cast<int>(Point_Camera.at(2)) <<")";
    // else
    //     ss<<"Null WTF";
    // putText(color_mat, ss.str(), cv::Point(Point_img.at(0), Point_img.at(1)), 0, 0.3, cv::Scalar(0, 255, 0));
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
void Objection::CheckStartPoint() {
    array<float,2> Pre_point;
   Pre_point.at(0)=Aera_Objection_R.x+Aera_Objection_R.width/2 ;
   Pre_point.at(1)=Aera_Objection_R.y+Aera_Objection_R.height/2;
//   cout<<Classname<<": Point_img:"<<Point_img.at(0)<<"  "<<Point_img.at(1)<<endl;
//   if (Enable== false) {
   Point_img.at(0)=Pre_point.at(0);
   Point_img.at(1)=Pre_point.at(1);
//   }
}
void Objection::Transform_ImgtoCam() {
//    Position_Transform Objection_center(Point_img,true);
//    Point_Camera=Objection_center.Report_PCL();
//    if(Point_Camera.at(2)<=200||Point_Camera.at(2)>=6000)
    if (Real_Point.size()>0)
       Enable= true;
    else Enable= false;
}
float Objection::Get_Area_Depth(cv::Rect Box) {
    std::array<int,Stride*Stride> Arr_Box;
    int result;
    for (int i = Box.y; i < Box.y+Box.height; ++i)
        for (int j = Box.x; j < Box.x+Box.width; ++j)
        {
            if (Depthmat.at<uint16_t>(i, j) > 6000 || Depthmat.at<uint16_t>(i, j) < 200)//D435有效探测距离有限 0.2M-6M
                Arr_Box.at((i - Box.y)*Stride + (j - Box.x))= 0;
            else
                Arr_Box.at((i - Box.y)*Stride + (j - Box.x)) = Depthmat.at<uint16_t>(i, j);
        }
    sort(Arr_Box.begin(),Arr_Box.end());
    for (auto i:Arr_Box){    //最小池化
        if (i>200){
            result=i;
            break;
        }
    }
    ///////////////////////////////平均池化
//    std::array<int,Stride*2> Value;
//    int sub=0;
//    for (int i = 0; i < Stride*2; ++i) {
//        Value.at(i)=Arr_Box.at(Stride*Stride/2-Stride+1+i);
//        if (Value.at(i)==0)
//            sub++;//剔除无效的点
//    }
//    result=std::accumulate(Value.begin(),Value.end(),0.0);
//    if (result<200)
//        result=0;
//    else
//        result=result/(Value.size()-sub);
    /////////////////////////////////////
    return result  ;
}
void Objection::DealRect() {
    ///3D点聚类算法
    ///目标区域稀疏化 获取稀疏化之后的点云信息
    if (Depthmat.size()==cv::Size(0,0)) {
        cout<<"Depthmat is null there is no topic pubulish return"<<endl;
        return;
    }
    //截取目标范围的深度图
    cv::Mat Object_Area_Depth = Depthmat(Area_Objection_D);
    cout<<"Object_Area_Depth : "<<Object_Area_Depth.size()<<endl;
    cout<<"Object_Area_info : "<<Object_Area_Depth<<endl;

    //稀疏化采集数据
    int height=0,width=0;
    for (int i = 0; i < Object_Area_Depth.rows-Stride; i += Stride) {
        height++;width=0;
        cout<<"3"<<endl;
        for (int j = 0; j < Object_Area_Depth.cols-Stride; j += Stride) {
            array<int, 2> Sparse_Point{(Area_Objection_D.x + j + Stride / 2)>(WidthCam-1) ? (WidthCam-1):(Area_Objection_D.x + j + Stride / 2), (Area_Objection_D.y + i + Stride / 2)>(HeightCam-1) ? (HeightCam-1):Area_Objection_D.y + i + Stride / 2};
            cout<<"Sparse_Point : "<<Sparse_Point.at(0)<<"  "<<Sparse_Point.at(1)<<endl;
            cv::Rect Area_ele(Area_Objection_D.x + j, Area_Objection_D.y + i, Stride, Stride);
            //获得该区域的距离
            cout<<"4"<<endl;
            auto Depth_value = Get_Area_Depth(Area_ele);//稀疏化
            cout<<"Depth value : "<<Depth_value<<endl;
            cout<<"Position"<<Sparse_Point.at(1)<<" "<<Sparse_Point.at(0)<<endl;//测试
            Depthmat.at<uint16_t>(Sparse_Point.at(1), Sparse_Point.at(0)) = Depth_value;
            cout<<"6"<<endl;
//            if (Depth_value > 0) {
            Objection_DepthPoint.push_back(Sparse_Point);
            auto IP=Position_Transform(Sparse_Point, false).Get_XYZ();
            Objection_PCL.push_back(IP);
            width++;
//            }
        }
    }


    int Long=Objection_PCL.size();
    cout<<"Test:"<<Long<<":"<<height*width<<endl;
    clock_t StartTime,EndTime;
    StartTime=clock();
    Algorithm_Objection_3D This_Algorithm(Objection_PCL,height,width);
    EndTime=clock();
    auto Time=(double)(EndTime - StartTime) / CLOCKS_PER_SEC;
    Point_Camera=This_Algorithm.Center_Point;
    Real_Point=This_Algorithm.Objection_3D;
}