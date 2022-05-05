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
    // cout<<"CD_Rotation"<<CD_Rotation<<endl;
    // cout<<"CD_Transation"<<CD_Transation<<endl;
    // cout<<"Pixel_P"<<Pix_P<<endl;
    Depth_cam = CD_Rotation.inverse() *(Pix_P - CD_Transation);
    // cout<<"Depth_cam "<<Depth_cam<<endl;
    // cout<<"Depth_inner"<<Depth_inner<<endl;
    Eigen::Vector3f D_m = Depth_inner*Depth_cam;
    // cout<<"D_m "<<D_m<<endl;
//    cout<<"C3:"<<Color_3(2)<<endl;
    Eigen::Vector2f Depth_Pix;
    Depth_Pix<< (D_m(0)/Color_cam(2)<(WidthCam-1) ? D_m(0)/Color_cam(2):(WidthCam-1) ) ,(D_m(1)/Color_cam(2)<(HeightCam-1)? D_m(1)/Color_cam(2): (HeightCam-1));//限制坐标宽度不要越界
    return Depth_Pix;
}

//获取深度图像坐标
std::array<int,2> Position_Transform::Get_Depth_Pixel(std::array<int,2> Pix){
    //彩色像素坐标到彩色相机坐标
    // cout<<"彩色像素坐标到彩色相机坐标 :"<<endl;
    Color_cam = Color_PixtoCamera(Pix);
    // cout<<"0.C_cam 彩色相机坐标:"<<Color_cam<<endl;
    //彩色相机坐标到深度像素坐标
    Eigen::Vector2f X_Y=Color_cameraToDepth_Pixel(Color_cam);
    // cout<<"1.X_Y 彩色相机坐标到深度像素坐标 :"<<X_Y<<endl;
    //深度图像坐标
    Depth_Pix={static_cast<int>(X_Y(0)),static_cast<int>(X_Y(1))};
    return Depth_Pix;
}


//返回相机坐标系下的位置
std::array<int, 3> Position_Transform::Get_XYZ() {
    Eigen::Vector3f Image_Pix;
    // cout<<"Depth_Pix:"<<Depth_Pix.at(0)<<" "<<Depth_Pix.at(1)<<endl;
    Image_Pix<<Depth_Pix.at(0),Depth_Pix.at(1),1.00;
    //取stride*stride的方块来求平均距离
    int stride = 0;
    int start_x = Depth_Pix.at(0) - stride;
    int start_y = Depth_Pix.at(1) - stride;
    int end_x = Depth_Pix.at(0) + stride;
    int end_y = Depth_Pix.at(1) + stride;
    int sum_count = 0;
    float distance_sum = 0;
    for(int i=start_x;i<=end_x;i++){
        for(int j=start_y;j<=start_y+stride*2;j++){
            if(Depthmat.at<uint16_t>(j,i)>0 && Depthmat.at<uint16_t>(j,i)<800){  //在[0，300]之间
                sum_count++;
                float temp_distance = Depthmat.at<uint16_t>(j,i);
                distance_sum+=temp_distance;
            }
        }
    }
    float average_distance;
    if(sum_count == 0){
        average_distance = 0;
    }else{
        average_distance = distance_sum/sum_count;
    }
    
    // cout<<"the distance:"<<average_distance<<endl;
    // cout<<"Image_Pix:"<<Image_Pix<<endl;
    // cout<<"Depth_inner"<<Depth_inner<<endl;
    Image_Pix=Depth_inner.inverse()*Image_Pix*average_distance;//深度像素转换为深度相机的相机坐标系，假设世界坐标系==相机坐标系，则外参就是个单位矩阵，所以忽略
    //modify
    RGB_Pix_position = Color_inner.inverse()*RGB_Pix_position*average_distance;
    PCL_Position.at(0)=RGB_Pix_position(0);
    PCL_Position.at(1)=RGB_Pix_position(1);
    PCL_Position.at(2)=RGB_Pix_position(2);

    // cout<<"result1 : "<<RGB_Pix_position(0) << " " << RGB_Pix_position(1) << " " << RGB_Pix_position(2) << endl;
    // cout<<"result2 : "<<Image_Pix(0) << " " << Image_Pix(1) << " " << Image_Pix(2) << endl;

    // PCL_Position.at(0)=Image_Pix(0);
    // PCL_Position.at(1)=Image_Pix(1);
    // PCL_Position.at(2)=Image_Pix(2);
    return PCL_Position;
}


//返回相机坐标系下的位置
Eigen::Vector3f Position_Transform::Get_ROBOT_TOOL_XYZ() {

    //0.相机坐标系到工具坐标系的变换矩阵
        /* 手眼标定得到的转移关系 是眼坐标到手坐标的关系 todo:load config */
        double qw = 0.7013088518485089;
        double qx = 0.0039751934245023735;
        double qy = -0.003477682492098677;
        double qz = 0.7128379885223908;
        // double tx = 62.9845508606165;
        // double ty = -32.21690881661964;
        double tx = 69.1845508606165;
        double ty = -30.68690881661964;
        double tz = -188.596799;


// translation: 
//   x: 0.0268392673326
//   y: -0.033216125011
//   z: -0.110592919824
// rotation: 
//   x: 0.0164554892007
//   y: 0.00674344927688
//   z: 0.716091924914
//   w: 0.697779404855

        // double tz = -209.596799;
        //旋转矩阵 初始化顺序，wxyz
        Eigen::Quaterniond q(qw,qx,qy,qz);
        q.normalize();
        Eigen::Matrix3d R = q.toRotationMatrix();
        //平移矩阵
        Eigen::Vector3d T = Eigen::Vector3d(tx,ty,tz);
        //相机坐标系到工具坐标系的变换矩阵
        Eigen::Matrix4d Trans_ObjToTool;
        Trans_ObjToTool.setIdentity();
        Trans_ObjToTool.block<3,3>(0,0) = R;
        Trans_ObjToTool.block<3,1>(0,3) = T;
    //1.工具坐标系到基坐标的的变换矩阵

        //获取当前机器人姿
        std::vector<double> current_xarm_state =   dataman::GetInstance()->GetXarmState();

        // std::cout<<"机器人当前位姿"<<current_xarm_state[0]<<","<<current_xarm_state[1]<<","<<current_xarm_state[2]<<std::endl;

        Eigen::Vector3d ea(current_xarm_state[5], current_xarm_state[4], current_xarm_state[3]);  //0 1 2 对应 z y x
        Eigen::Matrix3d R_2;
        R_2 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
        // cout<<"R_2:"<<R_2<<endl;
        Eigen::Vector3d T_2 = Eigen::Vector3d(current_xarm_state[0], current_xarm_state[1], current_xarm_state[2]);//当前的pose
        Eigen::Matrix4d Trans_ToolToBase; // Your Transformation Matrix
        Trans_ToolToBase.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
        Trans_ToolToBase.block<3,3>(0,0) = R_2;
        Trans_ToolToBase.block<3,1>(0,3) = T_2;
    //2.相机坐标系到基坐标系
        Eigen::Matrix<double,4,4> matrix_ObjToBase;
        matrix_ObjToBase=Trans_ToolToBase*Trans_ObjToTool;
        // cout<<"cam_to_base"<<endl<<matrix_ObjToBase<<endl;
    //3.得到基坐标系下的坐标
        Eigen::Vector4d result;
        Eigen::Vector4d ObjPosition;
        ObjPosition<<PCL_Position.at(0),PCL_Position.at(1),PCL_Position.at(2),1;
        result= matrix_ObjToBase*ObjPosition;
        Eigen::Vector3f temp2;
        temp2<<result(0),result(1),result(2);
        // cout<<"输出结果"<<result(0)<<","<<result(1)<<","<<result(2)<<endl;
        return temp2;
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