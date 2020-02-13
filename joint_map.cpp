#include <iostream>
#include <opencv2/core.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include<fstream>
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <tf/tf.h>


struct Pose
{
    /* data */
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
};

int main( int argc, char** argv )
{
       //提取一张图片,然后遍历这个图片上的所有的数据,存储下来,然后吧这个数据都变成0;
    //@1读取图片5张彩色图片+深度图像+txt
    cv::Mat color_image, depth_image;
    //然后将所有的图片都存储成为
    std::vector<cv::Mat> color_images,depth_images;
    std::string color_image_path,depth_image_path;
    color_image_path="/home/davidhan/davidhan_project/d_slambook/color/";
    depth_image_path="/home/davidhan/davidhan_project/d_slambook/depth/";
    std::ifstream  pose_file;//std::ifstream 读取,std::ofstream写入
    std::string pose_path=argv[1];
        //我实例化一个四元数来存储,实例化一个xyz的vector3d的矩阵来存储xyz
    pose_file.open(pose_path);
    Pose curr_data;
    double tmp_data[7];
    std::vector<Pose> pose_datas;
    for(int i=1;i<6;i++)
    {
        std::stringstream col_ss,dep_ss;
        col_ss<<i<<".png";
        dep_ss<<i<<".pgm";
        std::string color_full_path=color_image_path+col_ss.str();
        color_image=cv::imread(color_full_path);
        std::cout<<"path:"<<color_full_path<<std::endl;
        color_images.push_back(color_image);
        std::string depth_full_path=depth_image_path+dep_ss.str();
        depth_image=cv::imread(depth_full_path,-1);//深度图像这边必须是-1
        depth_images.push_back(depth_image);
        //依次提取,这边的提取是按照顺序,不关心是否空格
        pose_file>>curr_data.x>>curr_data.y>>curr_data.z>>curr_data.qx>>curr_data.qy>>curr_data.qz>>curr_data.qw;
        std::cout<<"curr:"<<curr_data.x<<std::endl;
        pose_datas.push_back(curr_data);
    }
    pose_file.close();
    //@2 拼接点云
    double cx=325.5;
    double cy=253.6;
    double fx=518;
    double fy=519;
    double depth_scale=1000;//深度图像的尺度
    //这边的点云的公式是?
    //如果知道相机在世界坐标系下的位姿,那么,你就知道相机对应的点在世界坐标系下的位姿.
    //那么首先我要利用的相机在世界坐标系下的位姿态
    //如何根据相机的内参,来计算各个点在世界坐标系下的姿态呢?
    //数据准备好以后,利用
    // typedef pcl::PointXYZRGB PointT;
    // typedef pcl::PointCloud<PointT> pointcloud;
    // pointcloud::Ptr ptr(new pointcloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr RL0(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Matrix4d camera_in_word = Eigen::Matrix4d::Identity();//单位矩阵 4*4
    cv::Mat curr_color,curr_depth;
    double  depth_dis=0;
    for (int i=0 ;i<5;i++)
    {
        //处理帧
        curr_depth=depth_images[i];
        curr_color=color_images[i]; 
        //定义一个变换矩阵
        camera_in_word(0,3)=pose_datas[i].x;
        camera_in_word(1,3)=pose_datas[i].y;
        camera_in_word(2,3)=pose_datas[i].z;
        tf::Quaternion quater_tmp(pose_datas[i].qx,pose_datas[i].qy,pose_datas[i].qz,pose_datas[i].qw);//x,y,z,w
        tf::Vector3 y_axis(1,0,0);
        //quater_tmp.setRotation(y_axis,M_PI);
        tf::Matrix3x3 Matrix_1;
         Matrix_1.setRotation(quater_tmp);
            tf::Vector3 v1_1,v1_2,v1_3;
            v1_1=Matrix_1[0];
            v1_2=Matrix_1[1];
            v1_3=Matrix_1[2];
            camera_in_word (0,0)=v1_1[0];
            camera_in_word (0,1)=v1_1[1];
            camera_in_word (0,2)=v1_1[2];
            camera_in_word (1,0)=v1_2[0];
            camera_in_word (1,1)=v1_2[1];
            camera_in_word (1,2)=v1_2[2];
            camera_in_word (2,0)=v1_3[0];
            camera_in_word (2,1)=v1_3[1];
            camera_in_word (2,2)=v1_3[2];
                //对所有的像素点进行处理
                std::cout<<"开始处理第一张图片"<<std::endl;
        for(int ii=0;ii<curr_color.cols;ii++)
        {
            for(int jj=0;jj<curr_color.rows;jj++)
           {
               pcl::PointXYZRGB  cloud_1; 
                //对每个像素点进行处理
                //提取出深度尺度,深度图每个点的像素数值,就是对应的距离值
                //在深度图相中提取当餐的深度信息
                unsigned int tmp  =curr_depth.ptr<unsigned  short>(jj)[ii];

               if(tmp==0) continue; //防止深度为0的现象
             //   std::cout<<"tmp:"<<tmp< <std::endl;
            // std::cout<<"d:"<<tmp<<std::endl;
             //两张照片是一样的结果,那么就
                depth_dis=(double)tmp/depth_scale;
                //depth_dis=1;
                Eigen::Vector4d point_in_camera;
             
                point_in_camera[2]=depth_dis;//z
                point_in_camera[0]=(ii-cx)*depth_dis/fx;//x 感触:u和v只是一个索引,是一个遍历的数值
                point_in_camera[1]=(jj-cy)*depth_dis/fy;//y 根本没有改变某个像素的数值
                point_in_camera[3]=1;
                //然后需要吧这些点转换到世界坐标系下,
                Eigen::Vector4d point_in_word;
                point_in_word=camera_in_word*point_in_camera;
                //
                cloud_1.x=point_in_word[0];
                cloud_1.y=-point_in_word[1];//为了pcl_viewer的显示, 相当与绕x轴旋转180度,
                cloud_1.z=-point_in_word[2];//如果等效与y这边取反和x轴取反向
                cloud_1.b=curr_color.data[jj*curr_color.step+ii*curr_color.channels()];
                cloud_1.g=curr_color.data[jj*curr_color.step+ii*curr_color.channels()+1];
                cloud_1.r=curr_color.data[jj*curr_color.step+ii*curr_color.channels()+2];
                RL0->points.push_back(cloud_1);
                //左乘一个  相机在世界坐标系下的位置 等效于 
                // 点在世界坐标系的位置  = 相机在世界坐标系下的位置* 点在相机坐标系下的位置
            }
       } 
     std::cout<<"处理完第一张图片"<<std::endl;
    }
    //pcl::PCDWriter writer;

		RL0->width = RL0->points.size();
		RL0->height = 1;
		RL0->is_dense = true;

    //writer.write("3vlp_result.pcd",cloud_result);
 //   cv::imshow("image",depth_images[0]);
    // cv::waitKey(0);
    std::cout<<"成功存储pcd"<<std::endl;
    pcl::io::savePCDFile("3vlp_result.pcd", *RL0);
    return 0;
}
