//
// Created by jzx
//
// 坐标转换，从相机坐标系到世界坐标系 
#include "Position_Transform.h"
#include "include.h"

//彩色图像像素坐标系到相机坐标系 (u,v)->(x,y,z)
Eigen::Vector3f Position_Transform::Color_PixtoCamera(std::array<int,2> Pix){
    Eigen::Vector3f Pix_3D ;
    Pix_3D<<Pix.at(0),Pix.at(1),1.00;
    cout<<"Pix_3D:"<<Pix_3D<<endl;
    cout<<"Color_inner:"<<Color_inner<<endl;
    Color_cam=Color_inner.inverse()*Pix_3D*1.00;//Depthmat.at<uint16_t>(Pix_P(0),Pix_P(1))
    cout<<"color_cam : "<<Color_cam<<endl;
    return  Color_cam;
}

//获取相机内参数和转换矩阵
void Position_Transform::Get_camera_referance() {
    CD_Rotation=dataman::GetInstance()->GetMTR();
    CD_Transation=dataman::GetInstance()->GetV_T();
    Depth_inner= dataman::GetInstance()->GetInnerTransformation_Depth();
    Color_inner=dataman::GetInstance()-> GetInnerTransformation_Color();
    Depthmat = dataman::GetInstance()->Getdepthmat();
    color_mat = dataman::GetInstance()->Getcolormat();
    
}

//彩色相机坐标到深度像素坐标 (x,y,z)->(u_d,v_d) 
Eigen::Vector2f Position_Transform::Color_cameraToDepth_Pixel(Eigen::Vector3f Pix_P) {
    Depth_cam = CD_Rotation.inverse() *(Pix_P - CD_Transation);
    Eigen::Vector3f D_m = Depth_inner*Depth_cam;
//    cout<<"C3:"<<Color_3(2)<<endl;
    Eigen::Vector2f Depth_Pix;
    Depth_Pix<< (D_m(0)/Color_cam(2)<(WidthCam-1) ? D_m(0)/Color_cam(2):(WidthCam-1) ) ,(D_m(1)/Color_cam(2)<(HeightCam-1)? D_m(1)/Color_cam(2): (HeightCam-1));//限制坐标宽度不要越界
    return Depth_Pix;
}

//根据深度图像坐标获取深度距离
std::array<int,2> Position_Transform::Get_Depth_Pixel(std::array<int,2> Pix){
    //彩色像素坐标到彩色相机坐标
    Color_cam = Color_PixtoCamera(Pix);
    //彩色相机坐标到深度像素坐标
    Eigen::Vector2f X_Y=Color_cameraToDepth_Pixel(Color_cam);
    //深度图像坐标
    Depth_Pix={static_cast<int>(X_Y(0)),static_cast<int>(X_Y(1))};
    return Depth_Pix;
}


//返回相机坐标系下的位置
std::array<int, 3> Position_Transform::Get_XYZ() {
    Eigen::Vector3f Image_Pix;
    cout<<"Depth_Pix:"<<Depth_Pix.at(0)<<" "<<Depth_Pix.at(1)<<endl;
    Image_Pix<<Depth_Pix.at(0),Depth_Pix.at(1),1.00;
     Image_Pix<<100,100,1.00;
    Image_Pix=Depth_inner.inverse()*Image_Pix*Depthmat.at<uint16_t>(Depth_Pix.at(1),Depth_Pix.at(0));
    PCL_Position.at(0)=Image_Pix(0);
    PCL_Position.at(1)=Image_Pix(1);
    PCL_Position.at(2)=Image_Pix(2);
    return PCL_Position;
}

//构造函数
Position_Transform::Position_Transform(std::array<int,2> Pix,bool flag) {
    if (flag)//flag==true means RGB_Pix
    {
        //rgb像素坐标
        Depth_Pix=Get_Depth_Pixel(Pix);
    }
    else{
        //depth像素坐标
        Depth_Pix=Pix;
//        Depth_Pix.at(0)=Pix.at(0)<(WidthCam-1) ? Pix.at(0):(WidthCam-1) ;// false means Depth_Pix
//        Depth_Pix.at(1)=Pix.at(1)<(HeightCam-1) ? Pix.at(1):(HeightCam-1) ;// false means Depth_Pix
    }
    Get_camera_referance();//得到相机参数
}
Position_Transform::~Position_Transform() {
//    std::cout<<"Point destructor "<<std::endl;
}