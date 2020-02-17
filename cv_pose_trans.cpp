#include <iostream>
#include <opencv2/core.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>//基本图像运算
#include <opencv2/features2d/features2d.hpp>//用于特征点匹配
#include <opencv2/calib3d.hpp> //用于寻找本质矩阵
#include <eigen3/Eigen/Core>
#include<opencv2/core/eigen.hpp>//cv2eigen
#include <eigen3/Eigen/Geometry>//四元数
#include<sophus/se3.h>

#include<ceres/ceres.h>
#include <ceres/rotation.h>//用于将旋转矩阵进行旋转


//使用ceres常见报错:https://blog.csdn.net/SLAM_masterFei/article/details/79550923


void ceres_ba(const std::vector<cv::Point3d> pt3ds, const std::vector<cv::Point2d> pt2ds ,const cv::Mat &K, cv::Mat R, cv::Mat t );

struct d_CostFunctor
{
  d_CostFunctor(cv::Point3d p_ref, cv::Point2d p_cur,cv::Mat K):p_ref_(p_ref),p_cur_(p_cur)
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

    //  Eigen::AngleAxisd R_vector(0,rr_vec);
    //  Eigen::Matrix3d R_Matrix;
    //  R_Matrix=R_vector.toRotationMatrix();
    //  Eigen::Vector3d t_trans(1,1,1);
    //   //Eigen::Vector3d t_trans(t[0],t[1],t[2]);
    //  Sophus::SE3 R_T_Matrix(R_Matrix,t_trans);
    //  Eigen::Vector3d p_ref_in_ref_camera(p_ref_.x,p_ref_.y,p_ref_.z);
    //  Eigen::Vector3d p_ref_in_cur_camera;
    //  p_ref_in_cur_camera=R_T_Matrix*p_ref_in_ref_camera;
    //     double cx=325.1;
    // double cy=249.7;
    // double fx=517.3;
    // double fy=516.5;
   T u_in_curr_camera,v_in_curr_camera;
   u_in_curr_camera=T(517.3)*P_3d_in_curr[0]/P_3d_in_curr[2]+T(325.1);
  // u_in_curr_camera=T(0);
   v_in_curr_camera=T(516.5)*P_3d_in_curr[1]/P_3d_in_curr[2]+T(249.7);
  //  v_in_curr_camera=T(0);
   T u_curr=T(p_cur_.x);
   T v_curr=T(p_cur_.y);
    residual[0] = u_curr-u_in_curr_camera;
    residual[1] = v_curr-v_in_curr_camera;
    //也就是建立 之间的关系
     return true;
 }
 const cv::Point3d p_ref_;
 const cv::Point2d p_cur_;
 const cv::Mat K_;
};

int main( int argc, char** argv )
{
       //提取一张图片,然后遍历这个图片上的所有的数据,存储下来,然后吧这个数据都变成0;
    //@1读取图片
    cv::Mat source_image1, source_image2;
    source_image1=cv::imread("/home/davidhan/davidhan_project/tum/rgbd_dataset_freiburg1_desk/rgb/1305031453.359684.png");
    source_image2=cv::imread("/home/davidhan/davidhan_project/tum/rgbd_dataset_freiburg1_desk/rgb/1305031453.391690.png");
    //cv::imshow("233",source_image1);
    //cv::waitKey(0);
    //首先提取特征
    std::vector<cv::KeyPoint> kp1,kp2;
    cv::Ptr<cv::ORB> orb=cv::ORB::create(500,1.2,8); 
    orb->detect(source_image1,kp1);
    orb->detect(source_image2,kp2);
    cv::Mat Dep1,Dep2;
    orb->compute(source_image1,kp1,Dep1);
    orb->compute(source_image2,kp2,Dep2);
    //采用Dep来评估这两个图片
    std::vector<cv::DMatch> match12;
    cv::BFMatcher match_bf(cv::NORM_HAMMING2);
    match_bf.match(Dep1,Dep2,match12);
    //匹配结果放在match12当中
      std::cout<<"原始点的个数:size:"<<match12.size()<<std::endl;
    std::vector<cv::DMatch> good_match;
    //然后对好的特征点进行删选
     for(int i=0;i<kp1.size();i++)
  {
      if(match12[i].distance< 30)
      {
          good_match.push_back(match12[i]);
      }
  }
   std::cout<<"good match:size:"<<good_match.size()<<std::endl;//26个点
     cv::Mat image_show;
                  cv::drawMatches(source_image1,kp1,source_image2,kp2,good_match,image_show);
                   cv::imshow("good",image_show);
                   cv::waitKey(0);


   
   //方法一 2d-2d 对极约束 来求解从pose1->pose2的姿态变换 求解出位置1到位置2之间的变换
    double cx=325.1;
    double cy=249.7;
    double fx=517.3;
    double fy=516.5;
    double depth_scale=5000;//深度图像的尺度
  //  cv::Point2d camera_point(cx,cy);
  //  std::vector<cv::Point2d> pt1,pt2;
  //  cv::Mat essential_matrix;
  //   for(int i=0;i<kp1.size();i++)
  //   {
  //       pt1.push_back(kp1[match12[i].queryIdx].pt);//测试图像 放入的特征点好的点的个数
  //       pt2.push_back(kp2[match12[i].trainIdx].pt);//样本图像
  //   }
  //   essential_matrix=cv::findEssentialMat(pt1,pt2,fx,camera_point);
  //   cv::Mat R,t;
  //   cv::recoverPose(essential_matrix,pt1,pt2,R,t,fx,camera_point);
  //   //那么就是我通过求解本质矩阵,然后来恢复出了RT,得到了pose1->pose2时间的位姿变换
  //  // std::cout<<"R"<<R<<std::endl;
  //  // std::cout<<"t"<<t<<std::endl;   
  //   //验证办法
  //     Eigen::Matrix3d eigen_R;
  //     cv::cv2eigen(R,eigen_R);//通过这个方式实现cv的R转成eigen的R
  //     //拍摄这两张图片相机的位姿我是知道
  //     Eigen::Vector3d t1(-1.41952,-0.279885,1.43657),t2(-1.55819,-0.301094 ,1.6215),t12(0,0,0);
  //     Eigen::Quaterniond q1(-0.00926933, -0.222761, -0.0567118 ,0.973178),q2(-0.02707, -0.250946 ,-0.0412848 ,0.966741);
  //     Eigen::Quaterniond q12=q1.inverse()*q2;//四元数字相乘表示空间旋转
  //     t12=t2-t1;
  //     Eigen::Matrix3d result_R;
  //     //参考链接 https://blog.csdn.net/ktigerhero3/article/details/86244211
  //     result_R=q12.toRotationMatrix();
  //     std::cout<<"result_R:"<<result_R<<std::endl;
  //     std::cout<<"t12->从1位置变道2位置:"<<t12<<std::endl;//z轴方向上的误差挺大
  //     //@ 三角化,当我们有了相机的运动之后,那么我们需要估计出特征点的深度(特征点在空间当中的位置 )
  //    //通过三角化,我求解出的的深度.
  //    //那么我在这边的输入是什么?两个图像在归一化平面的坐标,
  //    //里面上面求解出来的pt1和pt2就是特征点的个数
  //    cv::Mat T1=(cv::Mat_<double> (3,4)<<
  //       1,0,0,0,
  //       0,1,0,0,
  //       0,0,1,0);
  //        cv::Mat T2=(cv::Mat_<double> (3,4)<<
  //       R.at<double>(0,0) , R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
  //        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
  //        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0));
  //        cv::Mat result_Tra;
  //        cv::triangulatePoints(T1,T2,pt1,pt2,result_Tra);//三角化求解出的点的深度信息
  //        std::cout<<"成功三角化"<<std::endl;

         //然后如何来计算重投影误差
         // @1 将所有的点转到非齐次,也就是说,将最后以为
         //遍历这个Mat类型
        //  //输出的是一列
        //  std::vector<cv::Point3d> gui_pts;
        //  for(int  i=0;i<result_Tra.cols;i++)
        //  {
        //    cv::Mat gui_resulit_Tra=result_Tra.col(i);//将这一列取出来,
        //    gui_resulit_Tra=gui_resulit_Tra/result_Tra.at<float>(3,0);
        //    cv::Point3d temp_p(
        //      gui_resulit_Tra.at<double>(0,0),
        //      gui_resulit_Tra.at<double>(1,0),
        //      gui_resulit_Tra.at<double>(2,0)
        //    );
        //    gui_pts.push_back(temp_p);
        //  //  std::cout<<gui_resulit_Tra<<std::endl;
        //  }
         //那么我现在有了第二张图片在世界坐标系下的位置
         //那么我如何求解出
         //这部分暂时还不是很理解

      //方法二: 3d-2d同样也是求解相机的姿态
      //我打算在路上最重要的事情,还是把一些算法的,吧slam这本书弄清楚.
      //p3p就是当我之后某个点的在空间当中的位置,然后推算出世界坐标系
      //首先我为了知道空间当中3d点的坐标,我希望载入深度图像
      Eigen::Vector3d cam1_point_in_word,cam2_point_in_word;
      std::vector<Eigen::Vector3d> cam1_pts,cam2_pts;
      cv::Mat depth_img1,depth_img2;
      depth_img1=cv::imread("/home/davidhan/davidhan_project/tum/rgbd_dataset_freiburg1_desk/depth/1305031453.374112.png",-1);
      //depth_img2=cv::imread(argv[4],-1);
      //然后对每张的图片的深度进行提取
      //然后计算每张图像的空间点的坐标
      //我只是找这26个点的深度信息
      // std::cout<<"开始读取深度图"<<std::endl;
      // for(int ii=0;ii<depth_img1.cols;ii++)
      // {
      //   for(int jj=0;jj<depth_img1.rows;jj++)
      //   {
      //       double dis1=depth_img1.ptr<ushort>(jj)[ii];
      //        if(dis1==0.0) continue;
      //        cam1_point_in_word[2]=dis1/depth_scale;//z
      //        cam1_point_in_word[0]=(ii-cx)/fx*cam1_point_in_word[2];//x
      //        cam1_point_in_word[1]=(jj-cy)/fy*cam1_point_in_word[2];//y
      //        cam1_pts.push_back(cam1_point_in_word);
      //   }
      // }
      // for(int ii=0;ii<depth_img2.cols;ii++)
      // {
      //   for(int jj=0;jj<depth_img2.rows;jj++)
      //   {
      //       double dis2=depth_img2.ptr<ushort>(jj)[ii];
      //        if(dis2==0.0) continue;
      //        cam2_point_in_word[2]=dis2/depth_scale;//z
      //        cam2_point_in_word[0]=(ii-cx)/fx*cam2_point_in_word[2];//x
      //        cam2_point_in_word[1]=(jj-cy)/fy*cam2_point_in_word[2];//y
      //        cam2_pts.push_back(cam2_point_in_word);
      //   }
      // }
      //现在我空间当中3个点的都是是到了,那么如何通过3d转到2d,然后求解相机姿态呢?
      //我现在只是需要这26个点的3d坐标
      //遍历所有描述子,什么是描述子呢?就是某个像素周围点的汉明距离,
      std::vector<cv::Point3d> d_pt3ds;
      std::vector<cv::Point2d> d_pt2ds;
      for(int i=0;i<kp1.size();i++)
      {
        double source_image1_depth=depth_img1.ptr<ushort>(int(kp1[i].pt.y))[int(kp1[i].pt.x)];
        //std::cout<<"第一幅图像的深度"<<source_image1_depth<<std::endl;
         if (source_image1_depth==0)
         {
           std::cout<<"深度为0"<<std::endl;
           continue;
         } 
          cv::Point3d pt12_match_dis;
        pt12_match_dis.x=(kp1[i].pt.x-cx)/fx*source_image1_depth/depth_scale;//算出对应的像素坐标
        pt12_match_dis.y=(kp1[i].pt.y-cy)/fy*source_image1_depth/depth_scale;
        pt12_match_dis.z=source_image1_depth/depth_scale;
        d_pt3ds.push_back(pt12_match_dis);
        cv::Point2d d(kp1[i].pt.x,kp1[i].pt.y);
       d_pt2ds.push_back(d);
      }
      std::cout<<"参考帧中的特征点:"<<d_pt3ds.size()<<std::endl;
      //std::cout<<"像素当中点:"<<d_pt2ds[1]<<std::endl  ;
      int test_pt_num=0;
      std::vector<cv::Point3d> pt12_3ds;
      std::vector<cv::Point2d> pt12_2ds;
      for(cv::DMatch m:good_match)
      {
        double pt12_depth=depth_img1.ptr<ushort>(int(kp1[m.queryIdx].pt.y))[int(kp1[m.queryIdx].pt.x)];
        if (pt12_depth==0) 
        {
          continue;
        }
        cv::Point3d pt12_match_dis;
        pt12_match_dis.x=(kp1[m.queryIdx].pt.x-cx)/fx*pt12_depth/depth_scale;//算出对应的像素坐标
        pt12_match_dis.y=(kp1[m.queryIdx].pt.y-cy)/fy*pt12_depth/depth_scale;
        pt12_match_dis.z=pt12_depth/depth_scale;
        pt12_3ds.push_back(pt12_match_dis);
        pt12_2ds.push_back(kp2[m.trainIdx].pt);
    
        //至少说明了一个问题, kp2[m.tranidx].pt是一个point2d的数据类型,然后m.trandix是一个索引
        //这项看看mtranidx是个什么东西
      }
     //????还是原来的老问题 queryIdx 回退trainsId之间有什么样子的区别
      std::cout<<"空间当中点的个数:"<<pt12_3ds.size()<<"以及点"<<pt12_3ds[1]<<std::endl;
      std::cout<<"像素点的个数:"<<pt12_2ds[1]<<std::endl;
      //定义内参矩阵k和r和t
      cv::Mat R_3d_2d,t_3d_2d;
      std::cout<<"开始检测内参矩阵"<<std::endl;
      cv::Mat K_3d_2d=(cv::Mat_<double>(3,3)<<fx,0,cx,0,fy,cy,0,0,1);
      std::cout<<"内参矩阵问题2"<<std::endl;
      std::cout<<"K:"<<K_3d_2d<<std::endl;
      //cv::solvePnP(pt12_3ds,pt12_2ds,K_3d_2d,cv::Mat(),R_3d_2d,t_3d_2d,false);
      cv::Mat inliers;
      //std::cout<<"算法的输入"<<pt12_3ds<<std::endl;

      cv::solvePnPRansac(pt12_3ds,pt12_2ds,K_3d_2d,cv::Mat(),R_3d_2d,t_3d_2d,false,100, 4.0, 0.99, inliers);//需要配置后面的参数,十分重要
      std::cout<<"内点的梳理"<<inliers.rows<<std::endl;

      cv::Mat result_3d_2d_R;
      cv::Rodrigues(R_3d_2d,result_3d_2d_R);

      std::cout<<"result_3d_2d_R:"<<result_3d_2d_R<<std::endl;
      std::cout<<"两张图片之间的平移量"<<t_3d_2d<<std::endl;

      cv::Mat ceres_R,ceres_t;
      ceres_t=t_3d_2d;
      ceres_R=R_3d_2d;
      ceres_ba(pt12_3ds,pt12_2ds,K_3d_2d,ceres_R,ceres_t);

      

      //3d->2d只需要一张图片的3d信息就可以了

      //利用opencv 当中的solve pnp来求解这个问题,那么
      //solve pnp这个函数你需要输入是什么? 输入是认为匹配好的3d点和2d点,相机的内参矩阵
     //那么就有一个问题:什么是ba?ba的输入是什么?ba的输出是什么?
     //通过ba来进行消除误差?

     //那么思考一下,这个问题,该如何用ceres来优化,优化的变量是什么?是优化成一条曲线还是?
     //这个问题跟优化有什么关系

    // cv::imshow("231",source_image1);
    // cv::waitKey(0);
    
    return 0;
}

void ceres_ba(const std::vector<cv::Point3d> pt3ds, const std::vector<cv::Point2d> pt2ds ,const cv::Mat &K, cv::Mat R, cv::Mat t )
{
  std::cout<<"输入的原始向量R"<<R<<std::endl;//是一个三维的旋转向量
  std::cout<<"输入的原始t"<<t<<std::endl;
  double R_input[3]={R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2)};
  double t_input[3]={t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0)};
  std::cout<<R_input[2]<<std::endl;
  std::cout<<t_input[2]<<std::endl;
  ceres::Problem d_pro;
  //对于每一个点都要优化
  std::cout<<"需要优化的点的数量"<<pt3ds.size()<<std::endl;
  for(int i=0;i<pt3ds.size();i++)
  {
    //输入的是一个u&v的位置
    //第一个参数是优化变量的个数,例如u & v 第二个参数是输入数组的维度
    //std::cout<<"第"<<i<<"个特征点"<<std::endl;
    //std::cout<<"pt:3d"<<pt3ds[i]<<std::endl;
    ceres::CostFunction *d_const_f=new ceres::AutoDiffCostFunction<d_CostFunctor,2,3,3>(new d_CostFunctor(pt3ds[i],pt2ds[i],K));
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
