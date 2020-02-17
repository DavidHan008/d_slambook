#include<iostream>
#include<fstream>
#include <opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgcodecs.hpp>

using namespace std;
using namespace cv;
void read_data(const char **argv,vector<Mat> rgbs,vector<Mat> depths);
void lk_follow();
 int main(int argc, const char** argv) {
     std::vector<cv::Mat> rgbs,depths;
    // read_data(argv,rgbs,depths);不能使用函数,因为会core_dump
      std::fstream ass_file_;
    ass_file_.open("/home/davidhan/davidhan_project/tum/rgbd_dataset_freiburg1_desk/associate.txt");//为了方便写的绝对路径
    std::string rgb_file,depth_file;
    vector<string> rgb_files,depth_files;
    double rgb_time,depth_time;
    vector<double>rgb_times,depth_times;
    //当你不知道的有多少行的时候
    std::cout<<"开始提取文件"<<std::endl;
     double cx=325.1;
    double cy=249.7;
    double fx=517.3;
    double fy=516.5;
    double depth_scale=5000;//深度图像的尺度
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
    Mat rgb,depth;
    string data_filename="/home/davidhan/davidhan_project/tum/rgbd_dataset_freiburg1_desk/";
    for(int i = 0; i<rgb_files.size();i++)
    {
        rgb=imread(data_filename+rgb_files[i]);
        depth=imread(data_filename+depth_files[i],-1);
        rgbs.push_back(rgb);
        depths.push_back(depth);
        // cv::imshow("good2",depth);
        // cv::waitKey(0);
    }
    // cv::Mat abc=rgbs[1];
    // cv::imshow("good",abc);
    // cv::waitKey(0);
     vector<cv::KeyPoint> kp_cur,kp_next;
     vector<cv::Point2f> keypoint_frist,keypoint_next;//因为在cv::KeyPoint当中存储就是point2f
     cv::Mat cur_image,frist_image;
     //对第i张进行处理
     for(int i=0;i<rgbs.size();i++)
     {
         //对第一张图片进行提取特征
         if(i==0)
         {
            cv::Ptr<cv::ORB> orb=cv::ORB::create(500,1.2,8);
            orb->detect(rgbs[i],kp_cur);//初始化的特征点
            for(auto kp:kp_cur)
            {
                keypoint_frist.push_back(kp.pt);
            }
            frist_image=rgbs[i];
            continue;
         }
         cur_image=rgbs[i];
        //  vector<uchar> cur_statue;
        //  vector<float> cur_error;
         //这部分是为了光流法
        //  std::cout<<"开始光流"<<std::endl;
        //  cv::calcOpticalFlowPyrLK(frist_image,cur_image,keypoint_frist,keypoint_next,cur_statue,cur_error);
        //          std::cout<<"结束光流"<<std::endl;
        // cv::Mat image_show=cur_image.clone();     
        // //画出所有的
        // for(cv::Point2f kp:keypoint_next)
        // {
        //     cv::circle(image_show,kp,10,cv::Scalar(0,255,0),5);//半径10 线宽5
        // }
        // cv::imshow("show",image_show);
        // cv::waitKey(0);
        
        //这部分是为了直接法
        //input: 相机内参矩阵K,然后和第一帧和各个帧之间的匹配关系 光流法的本质就是只对第一帧特征提取特征点
        

     }
    return 0;
}
void read_data(const char **argv,vector<Mat> rgbs,vector<Mat> depths)
{
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
    Mat rgb,depth;
    string data_filename="/home/davidhan/davidhan_project/tum/rgbd_dataset_freiburg1_desk/";
    for(int i = 0; i<rgb_files.size();i++)
    {
        rgb=imread(data_filename+rgb_files[i]);
        depth=imread(data_filename+depth_files[i],-1);
        rgbs.push_back(rgb);
        depths.push_back(depth);
        // cv::imshow("good2",depth);
        // cv::waitKey(0);
    }
}