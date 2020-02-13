#include"camera.h"
#include"frame.h"
#include "visual_odometry.h"
#include<fstream>
#include <opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgcodecs.hpp>
#include <opencv2/viz.hpp> //显示运动轨迹

 int main(int argc, const char** argv) {
    // myslam::Camera d_camera_;
     //传入的参数是
     Eigen::Vector3d t_(0,0,0);
     Eigen::Quaterniond q_(1,0,0,0);//tf当中是x y z w ,但是在eigen中是 w x y z
     Sophus::SE3 d_SE3(q_,t_);
    std::cout<<d_SE3.matrix()<<std::endl;
    Eigen::Vector3d tw_(1,1,1),result_(0,0,0);
    result_=d_SE3*tw_;
    std::cout<<result_<<std::endl;
    //开始可视化一些东西
    cv::viz::Viz3d vis("d_vo_camera_pose");
    cv::viz::WCoordinateSystem world_coor(1.0),camera_color(0.5);
    cv::Point3d camera_pos(0,-1.0,-1.0),camera_focal_point(0,0,0),camera_y_dir(0,1,0);
    cv::Affine3d camera_pose=cv::viz::makeCameraPose(camera_pos,camera_focal_point,camera_y_dir);
    vis.setViewerPose(camera_pose);
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH,2.0);
    camera_color.setRenderingProperty(cv::viz::LINE_WIDTH,1.0);
    vis.showWidget("World",world_coor);
    vis.showWidget("Camera",camera_color);



    cout<<"开始V0"<<endl;
    //读取图像,然后开始提取当前帧,
    //@1 提取相关的tum的文件,
    std::fstream ass_file_;
    ass_file_.open("/home/davidhan/davidhan_project/tum/rgbd_dataset_freiburg1_desk/associate.txt");//为了方便写的绝对路径
    std::string rgb_file,depth_file;
    vector<string> rgb_files,depth_files;
    double rgb_time,depth_time;
    vector<double>rgb_times,depth_times;
    //当你不知道的有多少行的时候
    std::cout<<"开始提取文件"<<std::endl;
    while(!ass_file_.eof())//注意这里一定是感叹号
    {
        ass_file_>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_files.push_back(rgb_file);
        depth_files.push_back(depth_file);
        rgb_times.push_back(rgb_time);
        depth_times.push_back(depth_time);
    }
    ass_file_.close();
    std::cout<<"关闭提取文件"<<std::endl;
    string data_filename="/home/davidhan/davidhan_project/tum/rgbd_dataset_freiburg1_desk/";
    //然后通过rgb_tiem
    vector<Mat> rgb_images,depth_images;
    //传入相机的参数
   myslam::Camera::Ptr ptr_camera(new myslam::Camera);//实例化一个指针
  // myslam::Camera  test_ptr=new myslam::Camera();
    //std::shared_ptr<int>  test_ptr(new int);
    //std::shared_ptr<myslam::Camera>  ptr_camera(new myslam::Camera);
    //myslam::Camera::Ptr ptr_camera;//实例化一个指针
    //这个需要注意一下,在实例化一个指针类型的写法
    ptr_camera->fx_=517.3;
    std::cout<<"实例化参数有问题"<<std::endl;
    ptr_camera->fy_=516.5;
    ptr_camera->cx_=325.1;
    ptr_camera->cy_=249.7;
    ptr_camera->depth_scale_=5000;
    //实例化一个vo
    std::cout<<"camera正常"<<std::endl;
    myslam::VisualOdometry::Ptr ptr_vo(new myslam::VisualOdometry); 
    std::cout<<"开始frame"<<std::endl;
    std::cout<<"总共有图片数量:"<<rgb_times.size()<<std::endl;
    for(int i=0;i<rgb_times.size();i++)
    {
        std::string name=data_filename+depth_files[i];
        std::cout<<"name:"<<name.c_str()<<std::endl;
        cv::Mat rgb_image=cv::imread(data_filename+rgb_files[i]);
        cv::Mat depth_image=cv::imread(data_filename+depth_files[i],-1);//提取深度图
        if(rgb_image.data==nullptr|| depth_image.data==nullptr)//不是由于深度图像导致的
        break;
        //rgb_images.push_back(rgb_image);
        //depth_images.push_back(depth_image);
          //实例化一个frame类型,提取依次建立当前帧和关键帧之间的联系
          //那么首先可以通过pnp建立特征点的匹配,然后
         myslam::Frame::Ptr ptr_frame (new myslam::Frame);
         ptr_frame->color_=rgb_image;
         ptr_frame->depth_=depth_image;
         ptr_frame->camera_=ptr_camera;
         ptr_frame->time_stamp_=rgb_times[i];//时间戳
        ptr_vo->addFrame(ptr_frame);
         //cv::imshow("image",rgb_image);
        // cv::waitKey(1);//等待1s
         //最后输出的变换矩阵
         //ptr_frame->T_c_w_
         std::cout<<"相机的变换矩阵:"<<ptr_frame->T_c_w_.inverse().matrix()<<std::endl;
         Sophus::SE3 Twc=ptr_frame->T_c_w_.inverse();
         //cv::Affine3d M;
         cv::Affine3d M(
             cv::Affine3d::Mat3(
                 Twc.rotation_matrix()(0,0),Twc.rotation_matrix()(0,1),Twc.rotation_matrix()(0,2),
                 Twc.rotation_matrix()(1,0),Twc.rotation_matrix()(1,1),Twc.rotation_matrix()(2,2),
                 Twc.rotation_matrix()(2,0),Twc.rotation_matrix()(2,1),Twc.rotation_matrix()(2,2)
             ),
             cv::Affine3d::Vec3(
                    Twc.translation()(0,0),Twc.translation()(1,0),Twc.translation()(2,0)
             )
         );
         //然后再进行可视化工具
         vis.setWidgetPose("Camera",M);
         vis.spinOnce(1,false);
    }
 


    




    return 0;
}