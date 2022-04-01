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
    Color_cam=Color_inner.inverse()*Pix_3D*1.00;//Depthmat.at<uint16_t>(Pix_P(0),Pix_P(1))"
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
    cout<<"CD_Rotation"<<CD_Rotation<<endl;
    cout<<"CD_Transation"<<CD_Transation<<endl;
    cout<<"Pixel_P"<<Pix_P<<endl;
    Depth_cam = CD_Rotation.inverse() *(Pix_P - CD_Transation);
    cout<<"Depth_cam "<<Depth_cam<<endl;
    cout<<"Depth_inner"<<Depth_inner<<endl;
    Eigen::Vector3f D_m = Depth_inner*Depth_cam;
    cout<<"D_m "<<D_m<<endl;
//    cout<<"C3:"<<Color_3(2)<<endl;
    Eigen::Vector2f Depth_Pix;
    Depth_Pix<< (D_m(0)/Color_cam(2)<(WidthCam-1) ? D_m(0)/Color_cam(2):(WidthCam-1) ) ,(D_m(1)/Color_cam(2)<(HeightCam-1)? D_m(1)/Color_cam(2): (HeightCam-1));//限制坐标宽度不要越界
    return Depth_Pix;
}

//获取深度图像坐标
std::array<int,2> Position_Transform::Get_Depth_Pixel(std::array<int,2> Pix){
    //彩色像素坐标到彩色相机坐标
    cout<<"彩色像素坐标到彩色相机坐标 :"<<endl;
    Color_cam = Color_PixtoCamera(Pix);
    cout<<"0.C_cam 彩色相机坐标:"<<Color_cam<<endl;
    //彩色相机坐标到深度像素坐标
    Eigen::Vector2f X_Y=Color_cameraToDepth_Pixel(Color_cam);
    cout<<"1.X_Y 彩色相机坐标到深度像素坐标 :"<<X_Y<<endl;
    //深度图像坐标
    Depth_Pix={static_cast<int>(X_Y(0)),static_cast<int>(X_Y(1))};
    return Depth_Pix;
}


//返回相机坐标系下的位置
std::array<int, 3> Position_Transform::Get_XYZ() {
    Eigen::Vector3f Image_Pix;
    cout<<"Depth_Pix:"<<Depth_Pix.at(0)<<" "<<Depth_Pix.at(1)<<endl;
    Image_Pix<<Depth_Pix.at(0),Depth_Pix.at(1),1.00;
    //取stride*stride的方块来求平均距离
    int stride = 1;
    int start_x = Depth_Pix.at(0) - stride;
    int start_y = Depth_Pix.at(1) - stride;
    int end_x = Depth_Pix.at(0) + stride;
    int end_y = Depth_Pix.at(1) + stride;
    int sum_count = 0;
    float distance_sum = 0;
    for(int i=start_x;i<=end_x;i++){
        for(int j=start_y;j<=start_y+stride*2;j++){
            if(Depthmat.at<uint16_t>(j,i)>0){
                sum_count++;
                float temp_distance = Depthmat.at<uint16_t>(j,i);
                distance_sum+=temp_distance;
            }
        }
    }
    float average_distance = distance_sum/sum_count;
    cout<<"the distance:"<<average_distance<<endl;
    cout<<"Image_Pix:"<<Image_Pix<<endl;
    cout<<"Depth_inner"<<Depth_inner<<endl;
    Image_Pix=Depth_inner.inverse()*Image_Pix*average_distance;//深度像素转换为深度相机的相机坐标系，假设世界坐标系==相机坐标系，则外参就是个单位矩阵，所以忽略
    //modify
    
    RGB_Pix_position = Color_inner.inverse()*RGB_Pix_position*average_distance;
    // PCL_Position.at(0)=RGB_Pix_position(0);
    // PCL_Position.at(1)=RGB_Pix_position(1);
    // PCL_Position.at(2)=RGB_Pix_position(2);

    cout<<"result1 : "<<RGB_Pix_position(0) << " " << RGB_Pix_position(1) << " " << RGB_Pix_position(2) << endl;
    cout<<"result2 : "<<Image_Pix(0) << " " << Image_Pix(1) << " " << Image_Pix(2) << endl;

    PCL_Position.at(0)=Image_Pix(0);
    PCL_Position.at(1)=Image_Pix(1);
    PCL_Position.at(2)=Image_Pix(2);
    return PCL_Position;
}


//返回相机坐标系下的位置
Eigen::Vector3f Position_Transform::Get_ROBOT_TOOL_XYZ() {
    /* 手眼标定得到的转移关系 是眼坐标到手坐标的关系 todo load config */

    double qw = 0.7013088518485089;
    double qx = 0.0039751934245023735;
    double qy = -0.003477682492098677;
    double qz = 0.7128379885223908;
    double tx = 0.0629845508606165;
    double ty = -0.03221690881661964;
    double tz = 0.019403200790438897;

    //旋转矩阵 初始化顺序，wxyz
    Eigen::Quaterniond q(qw,qx,qy,qz);
    q.normalize();
    Eigen::Matrix3d R = q.toRotationMatrix();

    
    //平移矩阵
    Eigen::Vector3d T(tx,ty,tz);

    //相机坐标系到工具坐标系的变换
    Eigen::Vector3f camera_pos;
    camera_pos<<PCL_Position.at(0),PCL_Position.at(1),PCL_Position.at(2);

    Eigen::Vector3f tool_position = R*camera_pos+T;


    return tool_position;
}


//构造函数
Position_Transform::Position_Transform(std::array<int,2> Pix,bool flag) {
    Get_camera_referance();//得到相机参数
    if (flag)//flag==true means RGB_Pix
    {
        //rgb像素坐标
        Depth_Pix=Get_Depth_Pixel(Pix);
        RGB_Pix_position<<Pix.at(0),Pix.at(1),1.00;
    }
    else{
        //depth像素坐标
        Depth_Pix=Pix;
//        Depth_Pix.at(0)=Pix.at(0)<(WidthCam-1) ? Pix.at(0):(WidthCam-1) ;// false means Depth_Pix
//        Depth_Pix.at(1)=Pix.at(1)<(HeightCam-1) ? Pix.at(1):(HeightCam-1) ;// false means Depth_Pix
    }

}
Position_Transform::~Position_Transform() {
//    std::cout<<"Point destructor "<<std::endl;
}