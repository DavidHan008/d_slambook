#include<iostream>
#include<fstream>
#include <opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgcodecs.hpp>
#include<ceres/ceres.h>
#include <ceres/rotation.h>//用于将旋转矩阵进行旋转
#include<ceres/types.h>

using namespace std;
using namespace cv;

//参考博客.这个人自己写了一个,感觉听牛逼
//https://blog.csdn.net/u012348774/article/details/83930476
//https://blog.csdn.net/qq_17032807/article/details/85265620

struct d_CostFunctor
{
  d_CostFunctor(cv::Point3d p_ref, float gray_curr,cv::Mat gray_image):p_ref_(p_ref),gray_curr_(gray_curr),gray_image_(gray_image)
  {  }
 template <typename T>
 bool operator()(const T* const R,const T* const t,T* residual)const{
  //通过旋转向量恢复出旋转矩阵,然后将3d点转化到当前帧的相机坐标系下
  T P_3d_in_ref[3]={T(p_ref_.x),T(p_ref_.y),T(p_ref_.z)};
  T P_3d_in_curr[3];
  ceres::AngleAxisRotatePoint(R,P_3d_in_ref,P_3d_in_curr);
  P_3d_in_curr[0]+=t[0];
  P_3d_in_curr[1]+=t[1];
  P_3d_in_curr[2]+=t[2];
  T u_in_curr_camera,v_in_curr_camera;
   u_in_curr_camera=T(517.3)*P_3d_in_curr[0]/P_3d_in_curr[2]+T(325.1);
   v_in_curr_camera=T(516.5)*P_3d_in_curr[1]/P_3d_in_curr[2]+T(249.7);
//   double u,v;
//   ceres::Jet<double, 6> abc;
//    u=const_cast<double*>(u_in_curr_camera);
//    v=const_cast<double*>(v_in_curr_camera);
//    double gray_du;
    T   gray_du=T(0);
        T gray_du_cur=T(gray_curr_);
      residual[0] =T(gray_du) -gray_du_cur;
     return true;
 }
 const cv::Point3d p_ref_;
 const double gray_curr_;
 const cv::Mat gray_image_;
};


void ceres_ba(const std::vector<cv::Point3f> pt3ds,const std::vector<cv::Point2f> pt2ds, const std::vector<double>gray_scale_, cv::Mat gray_image_,const cv::Mat &K, cv::Mat R, cv::Mat t );

void read_data(const char **argv,vector<Mat> rgbs,vector<Mat> depths);
void lk_follow();
 int main(int argc, const char** argv) {
     std::vector<cv::Mat> rgbs,depths,gray_images;
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
        // rgb_times.push_back(rgb_time);
        // depth_times.push_back(depth_time);
    }
    ass_file_.close();
    std::cout<<"关闭提取文件"<<std::endl;
    Mat rgb,depth;
    string data_filename="/home/davidhan/davidhan_project/tum/rgbd_dataset_freiburg1_desk/";
    for(int i = 0; i<rgb_files.size();i++)
    {
        rgb=imread(data_filename+rgb_files[i]);
         cv::Mat gray_image;
         cvtColor(rgb, gray_image, COLOR_RGB2GRAY);
         depth=imread(data_filename+depth_files[i],-1);
         gray_images.push_back(gray_image);
         depths.push_back(depth);
        // cv::imshow("good2",depth);
        // cv::waitKey(0);
    }
    // cv::Mat abc=rgbs[1];
    // cv::imshow("good",abc);
    // cv::waitKey(0);
     vector<cv::KeyPoint> kp_cur,kp_next;
     vector<double> gray_scale;//第一帧图像,所有特征点的亮度数值;
     vector<cv::Point2f> keypoint_frist,keypoint_next;//因为在cv::KeyPoint当中存储就是point2f
     cv::Mat cur_image,frist_image;
     cv::Mat R_curr,t_curr,k_curr;
     R_curr=cv::Mat::zeros(1,3, CV_32F);
     k_curr=cv::Mat::zeros(3,3, CV_32F);
     t_curr=cv::Mat::zeros(3,1, CV_32F);
     std::vector<cv::Point3f> pt3ds_ref;
     //对第i张进行处理
     for(int i=0;i<gray_images.size();i++)
     {
         //对第一张图片进行提取特征
         if(i==0)
         {
             cv::Mat gray_image_pre=gray_images[i].clone();
            cv::Ptr<cv::ORB> orb=cv::ORB::create(500,1.2,8);
            orb->detect(gray_image_pre,kp_cur);//初始化的特征点
            for(auto kp:kp_cur)
            {
                cv::Point3f pt3d_now;
                double depth_curr=depths[i].ptr<ushort>(int(kp.pt.y))[int(kp.pt.x)];//中间提取像素的部分必须用int
                if(depth_curr==0) continue;
                pt3d_now.z=depth_curr/depth_scale;
                pt3d_now.y=(kp.pt.y-cy)*depth_curr/depth_scale/fy;
                pt3d_now.x=(kp.pt.x-cx)*depth_curr/depth_scale/fx;
                pt3ds_ref.push_back(pt3d_now);
                keypoint_frist.push_back(kp.pt);
                //然后提取每个特征点的灰度值,就是某一点的像素
                double gray_kp=gray_image_pre.ptr<ushort>(cvRound(kp.pt.y))[cvRound(kp.pt.x)];
                gray_scale.push_back(gray_kp);
             }
            frist_image=gray_image_pre.clone();
            continue;
         }
         cur_image=gray_images[i];
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
        //input: 1,相机内参矩阵K,2,第一帧图片的3d点和亮度数值,3,对于一张新来的照片,提取对应特征点的亮度,4,初始化旋转矩阵和t都是0
         ceres_ba(pt3ds_ref,keypoint_frist,gray_scale,cur_image,k_curr,R_curr,t_curr );

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
        cv::Mat gray_image;
         cvtColor(rgb, gray_image, COLOR_RGB2GRAY);
        depth=imread(data_filename+depth_files[i],-1);
        rgbs.push_back(gray_image);
        depths.push_back(depth);
        // cv::imshow("good2",depth);
        // cv::waitKey(0);
    }
}

void ceres_ba(const std::vector<cv::Point3f> pt3ds,const std::vector<cv::Point2f> pt2ds, const std::vector<double>gray_scale_, cv::Mat gray_image_,const cv::Mat &K, cv::Mat R, cv::Mat t )
{
    //做一部分数据预处理,提取对应特征点方面的数据
  std::cout<<"输入的原始向量R"<<R<<std::endl;//是一个三维的旋转向量
  std::cout<<"输入的原始t"<<t<<std::endl;
  double R_input[3]={R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2)};
  double t_input[3]={t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0)};
  ceres::Problem d_pro;
  //对于每一个点都要优化
  std::cout<<"需要优化的点的数量"<<pt3ds.size()<<std::endl;
  cv::Point3f pt3d_curr;
//   const ceres::Jet<double, 6> abc={0};
  
//   int ab_c[6]=const_cast<int>(abc);
  for(int i=0;i<pt2ds.size();i++)
  {
    //输入的是一个u&v的位置
    //第一个参数是优化变量的个数,例如u & v 第二个参数是输入数组的维度
    //std::cout<<"第"<<i<<"个特征点"<<std::endl;
    //std::cout<<"pt:3d"<<pt3ds[i]<<std::endl;
     double gray_curr=gray_image_.ptr<ushort>(cvRound(pt2ds[i].y))[cvRound(pt2ds[i].x)];
    ceres::CostFunction *d_const_f=new ceres::AutoDiffCostFunction<d_CostFunctor,1,3,3>(new d_CostFunctor(pt3ds[i],gray_curr,gray_image_));
    d_pro.AddResidualBlock(d_const_f,new ceres::CauchyLoss(0.5),R_input,t_input);
  }
    ceres::Solver::Options d_opt;
    d_opt.linear_solver_type=ceres::DENSE_QR;
    d_opt.minimizer_progress_to_stdout=true;
    ceres::Solver::Summary sum;
    std::cout<<"开始求解"<<std::endl;
    ceres::Solve(d_opt,&d_pro,&sum); 
    std::cout<<"求解出结果"<<std::endl;
    std::cout << sum.BriefReport() << "\n";//输出优化的简要信息
    R.at<double>(0,0)=R_input[0];
        R.at<double>(0,1)=R_input[1];
        R.at<double>(0,2)=R_input[2];
      t.at<double>(0,0)=t_input[0];
       t.at<double>(1,0)=t_input[1]; 
      t.at<double>(2,0)=t_input[2];
      std::cout<<"输出最小化重投影误差的结果"<<std::endl;
    std::cout<<"输出的优化后的R"<<R<<std::endl;
    std::cout<<"输出的优化后的t"<<t<<std::endl;
 }
