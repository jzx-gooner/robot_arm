//
// Created by jzx
//
#include "objection.h"
#include "Algorithm_Objection_3D.h"
#include "time.h"
#include "include.h"


using namespace std;


// float variance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
// {   
// 	float res = 0.0;//定义平均距离
// 	float var = 0.0;//定义方差
// 	float standard_deviation = 0.0;
// 	int n_points = 0;//定义记录点云数量
// 	int nres;//定义邻域查找数量
// 			 //vector是顺序容器的一种。vector 是可变长的动态数组
// 	std::vector<int> indices(2);//创建一个包含2个int类型数据的vector //创建一个动态数组，存储查询点近邻索引 //等价于这两行代码 using std::vector; vector<int> indices(2);
// 	std::vector<float> sqr_distances(2);//存储近邻点对应平方距离
// 	pcl::KdTreeFLANN<pcl::PointXYZ> tree;//以k-d tree方式查找
// 	tree.setInputCloud(cloud);
 
// 	for (size_t i = 0; i < cloud->size(); ++i)//循环遍历每一个点
// 	{
// 		// if (!pcl_isfinite(cloud->points[i].x))//pcl_isfinite函数返回一个布尔值，检查某个值是不是正常数值
// 		// {
// 		// 	continue;
// 		// }
// 		//Considering the second neighbor since the first is the point itself.
// 		// kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) 
// 		//这是执行 K 近邻查找的成员函数（其中，当k为1的时候，就是最近邻搜索。当k大于1的时候，就是多个最近邻搜索，此处k为2）
// 		//K为要搜索的邻居数量（k the number of neighbors to search for）
// 		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);//函数返回值（返回找到的邻域数量），return number of neighbors found
// 		if (nres == 2)//如果为两个点之间
// 		{
// 			res += sqrt(sqr_distances[1]);//sqrt()函数，返回sqr_distances[1]的开平方数
// 			//std::cout << "sqr_distances[1]：" << sqr_distances[1] << std::endl;//打印与临近点距离的平方值
// 			++n_points;
// 		}
// 	}
// 	std::cout << "nres：" << nres << std::endl;
// 	std::cout << "点云总数量n_points：" << n_points << std::endl;
// 	if (n_points != 0)
// 	{
// 		res /= n_points;
// 		for (size_t i = 0; i < cloud->size(); ++i)
// 		{
// 			// if (!pcl_isfinite(cloud->points[i].x))
// 			// {
// 			// 	continue;
// 			// }
// 			nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
// 			if (nres == 2)
// 			{
// 				var += pow(sqrt(sqr_distances[1]) - res, 2);
// 				++n_points;
// 			}
// 		}	
// 		if (n_points != 0)
// 		{
// 			var /= n_points;
// 			standard_deviation = sqrt(var);
// 		}
// 	}
// 	std::cout << "平均距离：" << res << std::endl;
// 	std::cout << "方差：" << var << std::endl;
// 	std::cout << "标准差：" << standard_deviation << std::endl;
// 	return res;
// }


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
    Position_Transform PT(array<int,2>{center_x,center_y}, true);
    std::array<int, 3> center_location=PT.Get_XYZ();//转换
    ostringstream center_ss;
    center_ss << "("<<static_cast<int>(center_location[0])<<","<<static_cast<int>(center_location[1])<<","<<static_cast<int>(center_location[2]) <<")";
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
    //         Position_Transform PT(array<int,2>{i,j}, true);
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
    // clock_t StartTime,EndTime;
    // StartTime=clock();
    // Algorithm_Objection_3D This_Algorithm(Objection_PCL,height,width);
    // EndTime=clock();
    // auto Time=(double)(EndTime - StartTime) / CLOCKS_PER_SEC;
    // Point_Camera=This_Algorithm.Center_Point;
    // Real_Point=This_Algorithm.Objection_3D;
}