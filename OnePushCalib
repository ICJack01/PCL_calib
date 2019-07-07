#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#include <pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include <math.h>
using namespace std;
using namespace Eigen;

int user_data;
void
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    pcl::PointXYZ o;
    o.x = 0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.2, "sphere", 0);
    std::cout << "i only run once" << std::endl;

}

void
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 0, 0, "text", 5);
    //FIXME: possible race condition here:
    user_data++;
}

bool next_iteration = false;
//设置键盘交互函数
void keyboardEvent(const pcl::visualization::KeyboardEvent &event,void *nothing)
{
  if(event.getKeySym() == "space" && event.keyDown())
  next_iteration = true;
}
        /*... 上述函数表示当键盘空格键按下时，才可执行ICP计算 ... */


//四元数转欧拉角
Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    cout << "Quaterniond2Euler result is:" <<endl;
    cout << "x = "<< euler[2] << endl ;
    cout << "y = "<< euler[1] << endl ;
    cout << "z = "<< euler[0] << endl << endl;
    
    return euler;
}

// 欧拉角转四元数
Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
 
    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
    cout << "Euler2Quaternion result is:" <<endl;
    cout << "x = " << q.x() <<endl;
    cout << "y = " << q.y() <<endl;
    cout << "z = " << q.z() <<endl;
    cout << "w = " << q.w() <<endl<<endl;
    return q;
}

// 旋转矩阵转四元数
Eigen::Quaterniond rotationMatrix2Quaterniond(Eigen::Matrix3d R)
{
    Eigen::Quaterniond q = Eigen::Quaterniond(R);
    q.normalize();
    cout << "RotationMatrix2Quaterniond result is:" <<endl;
    cout << "x = " << q.x() <<endl;
    cout << "y = " << q.y() <<endl;
    cout << "z = " << q.z() <<endl;
    cout << "w = " << q.w() <<endl<<endl;
    return q;
}

Eigen::Matrix3d Quaternion2RotationMatrix(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    //cout << "Quaternion2RotationMatrix result is:" <<endl;
    //cout << "R = " << endl << R << endl<< endl;
    return R;
}

int main (int argc, char** argv)
{
  //创建对象
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // M_Lidar2IMUPointXYZ格式
//  pcl::PointCloud<pcl::PCLPointCloud2>::Ptr cloud2 (new pcl::PCLPointCloud2); //PCLPointCloud2格式
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_sor (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_negative (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PCLPointCloud2>::Ptr cloud_seg(new pcl::PCLPointCloud2 ()); //尝试pclpointcloud2
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>); 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Lidar (new pcl::PointCloud<pcl::PointXYZ>); 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icpresult (new pcl::PointCloud<pcl::PointXYZ>);

  
                  //点云读取
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Lidar_up (new pcl::PointCloud<pcl::PointXYZ>); 
                            //点云读取
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Lidar_down (new pcl::PointCloud<pcl::PointXYZ>); 
                            //点云读取
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Lidar_left (new pcl::PointCloud<pcl::PointXYZ>); 
                            //点云读取
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Lidar_right (new pcl::PointCloud<pcl::PointXYZ>); 
                  //点云读取
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Lidar_up_back (new pcl::PointCloud<pcl::PointXYZ>);
    
                //点云读取
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Lidar_upcheck (new pcl::PointCloud<pcl::PointXYZ>); 
                    //点云读取
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Lidar_downcheck (new pcl::PointCloud<pcl::PointXYZ>); 
                //点云读取
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Lidar_leftcheck (new pcl::PointCloud<pcl::PointXYZ>); 
                //点云读取
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Lidar_rightcheck (new pcl::PointCloud<pcl::PointXYZ>); 
                    //点云读取
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Lidar_upcheck_backward (new pcl::PointCloud<pcl::PointXYZ>); 
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////// 创建文本
    ofstream outFile_up;//创建了一个ofstream 对象
    ofstream outFile_centre;//创建了一个ofstream 对象
    ofstream outFile_left;//创建了一个ofstream 对象
    ofstream outFile_right;//创建了一个ofstream 对象
    
  
  pcl::IterativeClosestPoint <pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaxCorrespondenceDistance(100);  
  icp.setTransformationEpsilon(1e-12); 
  icp.setEuclideanFitnessEpsilon(0.0001); 
  icp.setMaximumIterations(500);  
  
    // IMU在riegl中的坐标 
//   double IMU_in_RieO_x_forward = 428818.66280137 - 428818.6620; // IMU -RIEGL
//   double IMU_in_RieO_y_forward = 4434518.18374816 - 4434522.0084;
//   double IMU_in_RieO_z_forward = 51.64304104 - 50.07620 - 0.92; // 多减一个杆臂值高度
//   Eigen::Matrix3d IMUOrienM3_forward;
//   IMUOrienM3_forward = Quaternion2RotationMatrix(-0.0021259, 0.00230024, -0.01630482, -0.99986216); // 输入小车的orientation朝向四元数 ///////  

//   double IMU_in_RieO_x_backward = 428818.57096972 - 428818.6620; // IMU -RIEGL
//   double IMU_in_RieO_y_backward = 4434517.72535859 - 4434522.0084;
//   double IMU_in_RieO_z_backward = 51.64420333 - 50.07620 - 0.92;
//   Eigen::Matrix3d IMUOrienM3_backward;
//   IMUOrienM3_backward = Quaternion2RotationMatrix(-0.00217242, -0.00051215, -0.99983477, 0.01803893); // 输入小车的orientation朝向四元数 ///////
    // 前向IMU坐标设定
  Eigen::MatrixXd IMUBestPose_forward = MatrixXd::Identity(4, 4);
  IMUBestPose_forward(0,3) = 428818.357015953;
  IMUBestPose_forward(1,3) = 4434517.49785823;
  IMUBestPose_forward(2,3) = 51.64304104;
  Eigen::MatrixXd AttenaInIMU_forward = MatrixXd::Identity(4,4);
  AttenaInIMU_forward(0,3) = -0.297;
  AttenaInIMU_forward(1,3) = 0.0;
  AttenaInIMU_forward(2,3) = 0.92; 
  Eigen::Matrix3d IMUOrienM3_forward;
  IMUOrienM3_forward = Quaternion2RotationMatrix(-0.0021259, 0.00230024, -0.01630482, -0.99986216); // 输入小车的orientation朝向四元数 ///////
  Eigen::MatrixXd IMUOrienM4_forward = MatrixXd::Identity(4,4);
  IMUOrienM4_forward.topLeftCorner(3,3) = IMUOrienM3_forward;
  Eigen::MatrixXd ArmCorrection_forward = MatrixXd::Identity(4,4);
  ArmCorrection_forward = IMUOrienM4_forward.inverse() * ((-1) * AttenaInIMU_forward);
  double IMU_in_RieO_x_forward = IMUBestPose_forward(0,3) + ArmCorrection_forward(0,3) - 428818.6620; // IMU -RIEGL
  double IMU_in_RieO_y_forward = IMUBestPose_forward(1,3) + ArmCorrection_forward(1,3) - 4434522.0084;
  double IMU_in_RieO_z_forward = IMUBestPose_forward(2,3) + ArmCorrection_forward(2,3) - 50.07620;
    // 后向IMU坐标设定
  Eigen::MatrixXd IMUBestPose_backward = MatrixXd::Identity(4, 4);
  IMUBestPose_backward(0,3) = 428818.906923111;
  IMUBestPose_backward(1,3) = 4434517.56152473;
  IMUBestPose_backward(2,3) = 51.64420333;
  Eigen::MatrixXd AttenaInIMU_backward = MatrixXd::Identity(4,4);
  AttenaInIMU_backward(0,3) = -0.297;
  AttenaInIMU_backward(1,3) = 0.0;
  AttenaInIMU_backward(2,3) = 0.92; 
  Eigen::Matrix3d IMUOrienM3_backward;
  IMUOrienM3_backward = Quaternion2RotationMatrix(-0.00217242, -0.00051215, -0.99983477, 0.01803893);// 输入小车的orientation朝向四元数 ///////
  Eigen::MatrixXd IMUOrienM4_backward = MatrixXd::Identity(4,4);
  IMUOrienM4_backward.topLeftCorner(3,3) = IMUOrienM3_backward;
  Eigen::MatrixXd ArmCorrection_backward = MatrixXd::Identity(4,4);
  ArmCorrection_backward = IMUOrienM4_backward.inverse() * ((-1) * AttenaInIMU_backward);
  double IMU_in_RieO_x_backward = IMUBestPose_backward(0,3) + ArmCorrection_backward(0,3) - 428818.6620; // IMU -RIEGL
  double IMU_in_RieO_y_backward = IMUBestPose_backward(1,3) + ArmCorrection_backward(1,3) - 4434522.0084;
  double IMU_in_RieO_z_backward = IMUBestPose_backward(2,3) + ArmCorrection_backward(2,3) - 50.07620;
  
  double IMU_in_RieO_x;
  double IMU_in_RieO_y;
  double IMU_in_RieO_z;
  //雷达点云初值调整变换, 给ICP算法一个较好的初值
  pcl::io::loadPCDFile ("0.05m_qianxun_small.pcd", *cloud);
        
  float x ;
  float y ;
  float z;
  float rx;
  float ry;
  float rz;
  pcl::PassThrough <pcl::PointXYZ> pass;
  float xlim1;
  float xlim2;
  float ylim1;
  float ylim2;
  float zlim1;
  float zlim2;
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
  Eigen::Matrix4f transInit;
  transInit << 1,0,0,0,
  0,1,0,0,
  0,0,1,0,
  0,0,0,1;
    //3d 统计滤波器
  pcl::StatisticalOutlierRemoval <pcl::PointXYZ> sor;
  int MeanK = 20;
  float StddevMulThresh = 0.4;
  // 上雷达：U。 下雷达：D。 左雷达：L。 右雷达：R。
  char LidarPosition = 'R';   ////////////////////
  
    // 点云配准效果显示
  boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("icp test"));  //定义窗口共享指针
  int v1 ; //定义两个窗口v1，v2，窗口v1用来显示初始位置，v2用以显示配准过程
  view->createViewPort(0.0,0.0,1.0,1.0,v1);  //四个窗口参数分别对应x_min,y_min,x_max.y_max.
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color (cloud,0,250,0);  //目标点云为绿色
//   view->addPointCloud(cloud,target_cloud_color,"riegl_cloud_v1",v1); //将点云添加到v1窗口
  
  int R_value;
  int G_value;
  int B_value;
  
  Eigen::Matrix3d IMUOrienM3;
    // 建立IMU位姿描述
  Eigen::Vector4d IMUPosition; // 建立IMU
  Eigen::Matrix4d IMU_M4;
    // IMU到riegl的变换矩阵
  Eigen::Matrix4d M_IMU2RieO;
  Eigen::Matrix4d M_IMU2RieO_forward;
  Eigen::Matrix4d M_IMU2RieO_backward;
    // IMU到riegl的变换矩阵求逆
  Eigen::Matrix4d M_IMU2RieO_inv;
    // 求标定结果
  Eigen::Matrix4d M_Lidar2IMU; // 求激光雷达到IMU的点云变换矩阵
  Eigen::Matrix4d M_Lidar2IMU_0; // 求激光雷达到IMU的点云变换矩阵
  Eigen::Matrix4d M_Lidar2IMU_1;
  Eigen::Matrix4d M_Lidar2IMU_2;
  Eigen::Matrix4d M_Lidar2IMU_3;
  Eigen::Matrix4d M_Lidar2IMU_4;
  
  Eigen::Matrix4d M_back2for;
  Eigen::Matrix4f M_calibresult_transfered;
  
  Eigen::Matrix3d M_Lidar2IMU_3d; // 激光雷达到IMU的旋转矩阵3d
  Eigen::Matrix3d M_Lidar2IMU_3d_inv; // 激光雷达到IMU的旋转矩阵3d 反转
  Eigen::Quaterniond QuatLidar2IMU;
  
  for (int lidar_order = 0; lidar_order < 5; lidar_order++)
  {
        switch (lidar_order)
        {
            case 0:
                pcl::io::loadPCDFile ("up.pcd", *cloud_Lidar_up);
                cloud_Lidar = cloud_Lidar_up;
                sor.setInputCloud(cloud_Lidar);
                sor.setMeanK(MeanK);
                sor.setStddevMulThresh(StddevMulThresh);
                sor.filter(*cloud_Lidar); // 滤除离群点
  
                x = 0.0f;
                y = 0.0f;
                z = 0.0f;
                rx = 0.0f/180.0f*M_PI;
                ry = 0.0f/180.0f*M_PI;
                rz = 180.0f/180.0f*M_PI;
                xlim1 = -12.0;                
                xlim2 = 12.0;
                ylim1 = 2.0;
                ylim2 = 22.0;
                zlim1 = -3.0;
                zlim2 = 3.0; 
                
                R_value = 255;
                G_value = 0;
                B_value = 0;
                break;
            case 1:
                pcl::io::loadPCDFile ("down.pcd", *cloud_Lidar_down);
                cloud_Lidar = cloud_Lidar_down;
                sor.setInputCloud(cloud_Lidar);
                sor.setMeanK(MeanK);
                sor.setStddevMulThresh(StddevMulThresh);
                sor.filter(*cloud_Lidar); // 滤除离群点
  
                x = 0.0f;
                y = 0.0f;
                z = 0.0f;
                rx = 0.0f/180.0f*M_PI;
                ry = 0.0f/180.0f*M_PI;
                rz = -90.0f/180.0f*M_PI;
                xlim1 = -11.0;                
                xlim2 = 9.0;
                ylim1 = 3.0;
                ylim2 = 21.0;
                zlim1 = -2.0;
                zlim2 = 2.0;
                
                R_value = 0;
                G_value = 0;
                B_value = 255;
                break;
            case 2:
                pcl::io::loadPCDFile ("left.pcd", *cloud_Lidar_left);
                cloud_Lidar = cloud_Lidar_left;
                sor.setInputCloud(cloud_Lidar);
                sor.setMeanK(MeanK);
                sor.setStddevMulThresh(StddevMulThresh);
                sor.filter(*cloud_Lidar); // 滤除离群点
  
                x = 0.0f;
                y = 0.0f;
                z = 0.0f;
                rx = 0.0f/180.0f*M_PI;
                ry = 0.0f/180.0f*M_PI;
                rz = -90.0f/180.0f*M_PI;
                xlim1 = -11.0;                
                xlim2 = 8.5;
                ylim1 = 3.0;
                ylim2 = 21.0;
                zlim1 = -2.0;
                zlim2 = 2.5;
                
                R_value = 255;
                G_value = 0;
                B_value = 255;
                break;
            case 3:
                pcl::io::loadPCDFile ("right.pcd", *cloud_Lidar_right);
                cloud_Lidar = cloud_Lidar_right;
                sor.setInputCloud(cloud_Lidar);
                sor.setMeanK(MeanK);
                sor.setStddevMulThresh(StddevMulThresh);
                sor.filter(*cloud_Lidar); // 滤除离群点
  
                x = 0.0f;
                y = 0.0f;
                z = 0.0f;
                rx = 0.0f/180.0f*M_PI;
                ry = 0.0f/180.0f*M_PI;
                rz = -90.0f/180.0f*M_PI;
                xlim1 = -11.0;                
                xlim2 = 9.0;
                ylim1 = 2.0;
                ylim2 = 21.0;
                zlim1 = -2.0;
                zlim2 = 2.0;
                
                R_value = 255;
                G_value = 255;
                B_value = 255;
                break;
            case 4:
                pcl::io::loadPCDFile ("up_b_1.pcd", *cloud_Lidar_up);
                cloud_Lidar = cloud_Lidar_up;
                sor.setInputCloud(cloud_Lidar);
                sor.setMeanK(MeanK);
                sor.setStddevMulThresh(StddevMulThresh);
                sor.filter(*cloud_Lidar); // 滤除离群点
  
                x = 0.0f;
                y = -5.0f;
                z = 0.0f;
                rx = 0.0f/180.0f*M_PI;
                ry = 0.0f/180.0f*M_PI;
                rz = 0.0f/180.0f*M_PI;
                xlim1 = -12.0;                
                xlim2 = 12.0;
                ylim1 = 2.0;
                ylim2 = 22.0;
                zlim1 = -3.0;
                zlim2 = 3.0; 
                
                R_value = 255;
                G_value = 255;
                B_value = 0;
                break;
        }
        //transform_2.Identity();
        transform_2.matrix() = transInit;
        //cout<< "transform_2 \n"<<transform_2.matrix()<<endl;redeclaration
        transform_2.translation() << x, y, z;
        transform_2.rotate (Eigen::AngleAxisf (rz, Eigen::Vector3f::UnitZ()));  
        transform_2.rotate (Eigen::AngleAxisf (ry, Eigen::Vector3f::UnitY()));
        transform_2.rotate (Eigen::AngleAxisf (rx, Eigen::Vector3f::UnitX()));
        //std::cout << "transform_2 \n"<<transform_2.matrix() << std::endl;//打印矩阵
        pcl::transformPointCloud (*cloud_Lidar, *cloud_Lidar, transform_2);

        pass.setInputCloud(cloud_Lidar);  
        pass.setFilterFieldName("x");
        pass.setFilterLimits(xlim1, xlim2);  // x direct cut
        pass.filter(* cloud_Lidar);
        //std::cout<<"x filter done!"<< endl;
  
        pass.setInputCloud(cloud_Lidar);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(ylim1, ylim2); // y direct cut
        pass.filter(* cloud_Lidar);
        //std::cout<<"y filter done!"<< endl;
        
        pass.setInputCloud(cloud_Lidar);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(zlim1, zlim2); // z direct cut
        pass.filter(* cloud_Lidar);
        //std::cout<<"z filter done!"<< endl;

        icp.setInputCloud(cloud_Lidar);
        icp.setInputTarget(cloud);
        icp.align(*cloud_Lidar);

        switch (lidar_order)
        {
            case 0:
                IMU_in_RieO_x = IMU_in_RieO_x_forward;
                IMU_in_RieO_y = IMU_in_RieO_y_forward;
                IMU_in_RieO_z = IMU_in_RieO_z_forward;
                IMUOrienM3 = IMUOrienM3_forward;
                break;
            case 1:
                IMU_in_RieO_x = IMU_in_RieO_x_forward;
                IMU_in_RieO_y = IMU_in_RieO_y_forward;
                IMU_in_RieO_z = IMU_in_RieO_z_forward;
                IMUOrienM3 = IMUOrienM3_forward;
                break;
            case 2:
                IMU_in_RieO_x = IMU_in_RieO_x_backward;
                IMU_in_RieO_y = IMU_in_RieO_y_backward;
                IMU_in_RieO_z = IMU_in_RieO_z_backward;
                IMUOrienM3 = IMUOrienM3_backward;
                break;
            case 3:
                IMU_in_RieO_x = IMU_in_RieO_x_backward;
                IMU_in_RieO_y = IMU_in_RieO_y_backward;
                IMU_in_RieO_z = IMU_in_RieO_z_backward;
                IMUOrienM3 = IMUOrienM3_backward;
                break;
            case 4:
                IMU_in_RieO_x = IMU_in_RieO_x_backward;
                IMU_in_RieO_y = IMU_in_RieO_y_backward;
                IMU_in_RieO_z = IMU_in_RieO_z_backward;
                IMUOrienM3 = IMUOrienM3_backward;
        }
        //cout<<lidar_order<<" th ICP:"<<endl;
        //cout<<"has conveged:"<<icp.hasConverged()<<"score:"<<icp.getFitnessScore()<<endl;
        //cout<<"matrix:\n"<<icp.getFinalTransformation()<<endl;
        
        Eigen::Matrix4f icp_result_f = icp.getFinalTransformation();
        Eigen::Matrix4f pre_transfer_f = transform_2.matrix();
        Eigen::Matrix4d icp_result_d = icp_result_f.cast<double>();
        Eigen::Matrix4d pre_transfer_d = pre_transfer_f.cast<double>();
        Eigen::Matrix4d PCAlignment = icp_result_d * pre_transfer_d;
        //cout<<"Point cloud transfer matrix:\n"<< PCAlignment << endl;
        

        M_IMU2RieO.setIdentity();
        M_IMU2RieO.topLeftCorner(3,3) = IMUOrienM3;
        M_IMU2RieO(0,3) = IMU_in_RieO_x; // IMU_M4(0,3)-428818.6620;
        M_IMU2RieO(1,3) = IMU_in_RieO_y; // IMU_M4(1,3)-4434521.0084;
        M_IMU2RieO(2,3) = IMU_in_RieO_z; // IMU_M4(2,3)-50.0762;
  

        M_IMU2RieO_inv = M_IMU2RieO.inverse();
  
        // 求标定结果
        M_Lidar2IMU = M_IMU2RieO_inv * PCAlignment; // 计算位置描述矩阵/点云返回变换矩阵
        switch (lidar_order)
        {
            case 0:
                M_Lidar2IMU_0 = M_Lidar2IMU;
                M_IMU2RieO_forward = M_IMU2RieO;
                break;
            case 1:
                M_Lidar2IMU_1 = M_Lidar2IMU;
                M_IMU2RieO_forward = M_IMU2RieO;
                break;
            case 2:
                M_Lidar2IMU_2 = M_Lidar2IMU;
                M_IMU2RieO_backward = M_IMU2RieO;
                break;
            case 3:
                M_Lidar2IMU_3 = M_Lidar2IMU;
                M_IMU2RieO_backward = M_IMU2RieO;
                break;
            case 4:
                M_Lidar2IMU_4 = M_Lidar2IMU;
                M_IMU2RieO_backward = M_IMU2RieO;
                break;
        }
        cout<<"M_Lidar2IMU: \n"<<M_Lidar2IMU<<endl;
        cout<< "Calib result: Lidar to IMU Movement x y z: " << lidar_order <<" th result\n"<< M_Lidar2IMU.col(3) << "\n" << endl;
        M_Lidar2IMU_3d = M_Lidar2IMU.topLeftCorner(3, 3);
        QuatLidar2IMU = Eigen::Quaterniond(M_Lidar2IMU_3d); // 定义Lidar位置描述四元数
        QuatLidar2IMU.normalize();
        cout<<"Calib result: Lidar to IMU quaternion:" <<endl;
        cout << "x = " << QuatLidar2IMU.x() <<endl;
        cout << "y = " << QuatLidar2IMU.y() <<endl;
        cout << "z = " << QuatLidar2IMU.z() <<endl;
        cout << "w = " << QuatLidar2IMU.w() <<endl;
  }
  
    
  
/////////////////////////////////////////  上雷达显示
    pcl::io::loadPCDFile ("up_60.pcd", *cloud_Lidar_upcheck);

    transform_3.matrix() = M_Lidar2IMU_0.cast<float>();
    pcl::transformPointCloud (*cloud_Lidar_upcheck, *cloud_Lidar_upcheck, transform_3);
                //点云变换显示
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> up_cloud_color (cloud_Lidar_upcheck,250,0,0);  //目标点云为绿色
    view->addPointCloud(cloud_Lidar_upcheck,up_cloud_color,"target_cloud_v0",v1); //将点云添加到v1窗口
    
    M_calibresult_transfered = transform_3.matrix();
    cout<<"\n up \n"<<transform_3.matrix()<<endl;
    
    cout<< "Calib result: Lidar to IMU matrix: \n"<< M_calibresult_transfered.col(3) << "\n" << endl;
    M_Lidar2IMU_3d = M_calibresult_transfered.topLeftCorner(3, 3).cast<double>();
    QuatLidar2IMU = Eigen::Quaterniond(M_Lidar2IMU_3d); // 定义Lidar位置描述四元数
    QuatLidar2IMU.normalize();
    cout<<"Calib result: Lidar to IMU quaternion:" <<endl;
    cout << "x = " << QuatLidar2IMU.x() <<endl;
    cout << "y = " << QuatLidar2IMU.y() <<endl;
    cout << "z = " << QuatLidar2IMU.z() <<endl;
    cout << "w = " << QuatLidar2IMU.w() <<endl;
    outFile_up.open("rfans16_front_up_novatel_extrinsics.yaml");//outFile 与一个文本文件关联
    outFile_up<<fixed;    //小数点格式显示double
    outFile_up<<"# proj: +proj=utm +zone=50 +ellps=WGS84\n# scale:1.11177\n# (XXX) Manually adjusted"<<endl;
    outFile_up<<"header:\n  stamp:\n    secs: 1422601952\n    nsecs: 288805456\n  seq: 0\n  frame_id: novatel"<<endl;
    outFile_up<<"transform:\n  translation:"<<endl;
    outFile_up<<"    x: "<<M_calibresult_transfered(0,3)<<endl;
    outFile_up<<"    y: "<<M_calibresult_transfered(1,3)<<endl;
    outFile_up<<"    z: "<<M_calibresult_transfered(2,3)<<endl;
    outFile_up<<"  rotation:\n    x: "<<QuatLidar2IMU.x()<<endl;
    outFile_up<<"    y: "<<QuatLidar2IMU.y()<<endl;
    outFile_up<<"    z: "<<QuatLidar2IMU.z()<<endl;
    outFile_up<<"    w: "<<QuatLidar2IMU.w()<<endl;
    outFile_up<<"child_frame_id: rfans16_front_up"<<endl;
    outFile_up.close();    //使用完文本文件后要用close()方法将其关闭
    
/////////////////////////////////////////  上雷达backward转换求解
    pcl::io::loadPCDFile ("up_60.pcd", *cloud_Lidar_upcheck_backward);
    M_back2for = M_Lidar2IMU_0 * M_Lidar2IMU_4.inverse() ;
    transform_3.matrix() = M_back2for.cast<float>() * M_Lidar2IMU_4.cast<float>();
    cout<< "temp --------------------here \n"<<endl;
    cout<< M_back2for<<endl;
    cout<< transform_3.matrix() <<endl;
    pcl::transformPointCloud (*cloud_Lidar_upcheck_backward, *cloud_Lidar_upcheck_backward, transform_3);
                //点云变换显示
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> up_cloud_color_backward (cloud_Lidar_upcheck_backward,250,250,0);  //目标点云为绿色
    view->addPointCloud(cloud_Lidar_upcheck_backward,up_cloud_color_backward,"target_cloud_v4",v1); //将点云添加到v1窗口


    
    
    
/////////////////////////////////////////  下雷达显示
    pcl::io::loadPCDFile ("down_60.pcd", *cloud_Lidar_downcheck);
                
    transform_3.matrix() = M_Lidar2IMU_1.cast<float>();
    pcl::transformPointCloud (*cloud_Lidar_downcheck, *cloud_Lidar_downcheck, transform_3);
                //点云变换显示
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> down_cloud_color (cloud_Lidar_downcheck, 0, 0, 250);  //目标点云为绿色
    view->addPointCloud(cloud_Lidar_downcheck,down_cloud_color,"target_cloud_v1",v1); //将点云添加到v1窗口
    
    M_calibresult_transfered = transform_3.matrix();
    cout<<"\n down \n"<<transform_3.matrix()<<endl;
    
    
    cout<< "Calib result: Lidar to IMU matrix: \n"<< M_calibresult_transfered.col(3) << "\n" << endl;
    M_Lidar2IMU_3d = M_calibresult_transfered.topLeftCorner(3, 3).cast<double>();
    M_Lidar2IMU_3d_inv = M_Lidar2IMU_3d.inverse();
    QuatLidar2IMU = Eigen::Quaterniond(M_Lidar2IMU_3d); // 定义Lidar位置描述四元数
    QuatLidar2IMU.normalize();
    cout<<"Calib result: Lidar to IMU quaternion:" <<endl;
    cout << "x = " << QuatLidar2IMU.x() <<endl;
    cout << "y = " << QuatLidar2IMU.y() <<endl;
    cout << "z = " << QuatLidar2IMU.z() <<endl;
    cout << "w = " << QuatLidar2IMU.w() <<endl;
    outFile_centre.open("rfans16_front_center_novatel_extrinsics.yaml");//outFile 与一个文本文件关联
    outFile_centre<<fixed;    //小数点格式显示double
    outFile_centre<<"# proj: +proj=utm +zone=50 +ellps=WGS84\n# scale:1.11177\n# (XXX) Manually adjusted"<<endl;
    outFile_centre<<"header:\n  stamp:\n    secs: 1422601952\n    nsecs: 288805456\n  seq: 0\n  frame_id: novatel"<<endl;
    outFile_centre<<"transform:\n  translation:"<<endl;
    outFile_centre<<"    x: "<<M_calibresult_transfered(0,3)<<endl;
    outFile_centre<<"    y: "<<M_calibresult_transfered(1,3)<<endl;
    outFile_centre<<"    z: "<<M_calibresult_transfered(2,3)<<endl;
    outFile_centre<<"  rotation:\n    x: "<<QuatLidar2IMU.x()<<endl;
    outFile_centre<<"    y: "<<QuatLidar2IMU.y()<<endl;
    outFile_centre<<"    z: "<<QuatLidar2IMU.z()<<endl;
    outFile_centre<<"    w: "<<QuatLidar2IMU.w()<<endl;
    outFile_centre<<"child_frame_id: rfans16_front_center"<<endl;
    outFile_centre.close();    //使用完文本文件后要用close()方法将其关闭
/////////////////////////////////////////  左雷达显示
    pcl::io::loadPCDFile ("left_60.pcd", *cloud_Lidar_leftcheck);
                
    transform_3.matrix() = M_back2for.cast<float>() * M_Lidar2IMU_2.cast<float>();
    pcl::transformPointCloud (*cloud_Lidar_leftcheck, *cloud_Lidar_leftcheck, transform_3);
                //点云变换显示
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> left_cloud_color (cloud_Lidar_leftcheck,250, 0, 250);  //目标点云为绿色
    view->addPointCloud(cloud_Lidar_leftcheck,left_cloud_color,"target_cloud_v2",v1); //将点云添加到v1窗口
    
    M_calibresult_transfered = transform_3.matrix();
    cout<<"\n left \n"<<transform_3.matrix()<<endl;
    
    
    cout<< "Calib result: Lidar to IMU matrix: \n"<< M_calibresult_transfered.col(3) << "\n" << endl;
    M_Lidar2IMU_3d = M_calibresult_transfered.topLeftCorner(3, 3).cast<double>();
    M_Lidar2IMU_3d_inv = M_Lidar2IMU_3d.inverse();
    QuatLidar2IMU = Eigen::Quaterniond(M_Lidar2IMU_3d); // 定义Lidar位置描述四元数
    QuatLidar2IMU.normalize();
    cout<<"Calib result: Lidar to IMU quaternion:" <<endl;
    cout << "x = " << QuatLidar2IMU.x() <<endl;
    cout << "y = " << QuatLidar2IMU.y() <<endl;
    cout << "z = " << QuatLidar2IMU.z() <<endl;
    cout << "w = " << QuatLidar2IMU.w() <<endl;
    outFile_left.open("rfans16_rear_left_novatel_extrinsics.yaml");//outFile 与一个文本文件关联
    outFile_left<<fixed;    //小数点格式显示double
    outFile_left<<"# proj: +proj=utm +zone=50 +ellps=WGS84\n# scale:1.11177\n# (XXX) Manually adjusted"<<endl;
    outFile_left<<"header:\n  stamp:\n    secs: 1422601952\n    nsecs: 288805456\n  seq: 0\n  frame_id: novatel"<<endl;
    outFile_left<<"transform:\n  translation:"<<endl;
    outFile_left<<"    x: "<<M_calibresult_transfered(0,3)<<endl;
    outFile_left<<"    y: "<<M_calibresult_transfered(1,3)<<endl;
    outFile_left<<"    z: "<<M_calibresult_transfered(2,3)<<endl;
    outFile_left<<"  rotation:\n    x: "<<QuatLidar2IMU.x()<<endl;
    outFile_left<<"    y: "<<QuatLidar2IMU.y()<<endl;
    outFile_left<<"    z: "<<QuatLidar2IMU.z()<<endl;
    outFile_left<<"    w: "<<QuatLidar2IMU.w()<<endl;
    outFile_left<<"child_frame_id: rfans16_rear_left"<<endl;
    outFile_left.close();    //使用完文本文件后要用close()方法将其关闭    
/////////////////////////////////////////  右雷达显示
    pcl::io::loadPCDFile ("right_60.pcd", *cloud_Lidar_rightcheck);
                
    transform_3.matrix() = M_back2for.cast<float>() * M_Lidar2IMU_3.cast<float>();
    pcl::transformPointCloud (*cloud_Lidar_rightcheck, *cloud_Lidar_rightcheck, transform_3);
    
                //点云变换显示
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> right_cloud_color (cloud_Lidar_rightcheck,250,250,250);  //目标点云为绿色
    view->addPointCloud(cloud_Lidar_rightcheck,right_cloud_color,"target_cloud_v3",v1); //将点云添加到v1窗口

    M_calibresult_transfered = transform_3.matrix();
    cout<<"\n right \n"<<transform_3.matrix()<<endl;
    
    
    cout<< "Calib result: Lidar to IMU matrix: \n"<< M_calibresult_transfered.col(3) << "\n" << endl;
    M_Lidar2IMU_3d = M_calibresult_transfered.topLeftCorner(3, 3).cast<double>();
    M_Lidar2IMU_3d_inv = M_Lidar2IMU_3d.inverse();
    QuatLidar2IMU = Eigen::Quaterniond(M_Lidar2IMU_3d); // 定义Lidar位置描述四元数
    QuatLidar2IMU.normalize();
    cout<<"Calib result: Lidar to IMU quaternion:" <<endl;
    cout << "x = " << QuatLidar2IMU.x() <<endl;
    cout << "y = " << QuatLidar2IMU.y() <<endl;
    cout << "z = " << QuatLidar2IMU.z() <<endl;
    cout << "w = " << QuatLidar2IMU.w() <<endl;
    outFile_right.open("rfans16_rear_right_novatel_extrinsics.yaml");//outFile 与一个文本文件关联
    outFile_right<<fixed;    //小数点格式显示double
    outFile_right<<"# proj: +proj=utm +zone=50 +ellps=WGS84\n# scale:1.11177\n# (XXX) Manually adjusted"<<endl;
    outFile_right<<"header:\n  stamp:\n    secs: 1422601952\n    nsecs: 288805456\n  seq: 0\n  frame_id: novatel"<<endl;
    outFile_right<<"transform:\n  translation:"<<endl;
    outFile_right<<"    x: "<<M_calibresult_transfered(0,3)<<endl;
    outFile_right<<"    y: "<<M_calibresult_transfered(1,3)<<endl;
    outFile_right<<"    z: "<<M_calibresult_transfered(2,3)<<endl;
    outFile_right<<"  rotation:\n    x: "<<QuatLidar2IMU.x()<<endl;
    outFile_right<<"    y: "<<QuatLidar2IMU.y()<<endl;
    outFile_right<<"    z: "<<QuatLidar2IMU.z()<<endl;
    outFile_right<<"    w: "<<QuatLidar2IMU.w()<<endl;
    outFile_right<<"child_frame_id: rfans16_rear_right"<<endl;
    outFile_right.close();    //使用完文本文件后要用close()方法将其关闭

  
    int iterations = 0; //迭代次数
    while(!view->wasStopped())
    {
        view->spinOnce();  //运行视图
        if (next_iteration)
        {
                  //icp.align(*cloud_Lidar);  //icp计算                
            cout<<"has conveged:"<<icp.hasConverged()<<"score:"<<icp.getFitnessScore()<<endl;
            cout<<"matrix:\n"<<icp.getFinalTransformation()<<endl;
            cout<<"iteration = "<<++iterations << endl;
            //如果icp.hasConverged=1,则说明本次配准成功，icp.getcloud_icpresultTransformation()可输出变换矩阵
            if (iterations == 50)  //设置最大迭代次数
            break;
            //view->updatePointCloud(cloud,aligend_cloud_color,"aligend_cloud_v2");
        }
        next_iteration = false;  //本次迭代结束，等待触发
    }
    return (0);
}
  
  
  ///////////////////////////////////////////////////////////////////////////
 
