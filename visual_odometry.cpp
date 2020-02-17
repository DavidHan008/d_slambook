#include"visual_odometry.h"


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

void ceres_ba(const std::vector<cv::Point3d> pt3ds, const std::vector<cv::Point2d> pt2ds ,const cv::Mat &K, cv::Mat R, cv::Mat t );


namespace myslam
{
    bool matIsEqual(const cv::Mat mat1, const cv::Mat mat2) {
	if (mat1.empty() && mat2.empty()) {
		return true;
	}
	if (mat1.cols != mat2.cols || mat1.rows != mat2.rows || mat1.dims != mat2.dims||
		mat1.channels()!=mat2.channels()) {
		return false;
	}
	if (mat1.size() != mat2.size() || mat1.channels() != mat2.channels() || mat1.type() != mat2.type()) {
		return false;
	}
	int nrOfElements1 = mat1.total()*mat1.elemSize();
	if (nrOfElements1 != mat2.total()*mat2.elemSize()) return false;
	bool lvRet = memcmp(mat1.data, mat2.data, nrOfElements1) == 0;
	return lvRet;
     }


     VisualOdometry::VisualOdometry()
     {
         orb_ = cv::ORB::create(500,1.2,8);//初始化特征点提取
          state_=INITIALIZING;
          min_dist=30;
     }
     VisualOdometry:: ~ VisualOdometry()
     {
         
     }
    bool VisualOdometry::addFrame( Frame::Ptr frame )//因为这边传入的参数必须是一个指针
    {
        //当第一帧来的时候进行特征点的匹配
        //然后根据不同的状态,进行判断
        switch (state_)
        {
        case INITIALIZING:
            /* code */
            {
                pts_3d_ref_.clear();
                state_=OK;
                curr_=ref_=frame;
                //提取当前帧的特征点
                std::cout<<"开始提取特征点"<<std::endl;
                orb_->detect(curr_->color_, keypoints_curr_);
                orb_->compute(curr_->color_,keypoints_curr_,descriptors_curr_);
                descriptors_ref_=cv::Mat();

                vector<cv::Point3d>  pt_curr_in_cameras;
                vector<cv::Point2d>  pt_curr_in_pixels;
                for(int i=0;i<keypoints_curr_.size();i++)
                {
                    double curr_depth=curr_->depth_.ptr<ushort>(int(keypoints_curr_[i].pt.y))[int(keypoints_curr_[i].pt.x)];
                    //计算特征点的像素和3d点
                   // std::cout<<"kp1当中提取的深度:"<<curr_depth<<std::endl;
                    //  if(curr_depth>0) 
                    //  {
                    //     std::cout<<"深度数值为0"<<std::endl;
                    //     continue;
                        Eigen::Vector2d u_v(keypoints_curr_[i].pt.x,keypoints_curr_[i].pt.y);
                        cv::Point2d curr_u_v(u_v(0,0),u_v(1,0));
                        Eigen::Vector3d point_in_camera_=curr_->camera_->pixel2camera(u_v,curr_depth);
                        cv::Point3d Pt_in_camera(point_in_camera_(0,0),point_in_camera_(1,0),point_in_camera_(2,0));
                        pt_curr_in_cameras.push_back(Pt_in_camera);
                        pt_curr_in_pixels.push_back(curr_u_v);
                        descriptors_ref_.push_back(descriptors_curr_.row(i));//描述子的问题,个人看法是因为特征点的id跟描述子的id不匹配
                    //  }
                }
             // std::cout<<"空间当中点的数量"<<pt_curr_in_cameras<<std::endl;
             // std::cout<<"像素中点:"<<pt_curr_in_pixels[1]<<endl  ;
                //更新相关的描述子和特征点
              //  descriptors_ref_=descriptors_curr_;// 很重要  描述子当前帧给了参考帧  不可以这么做
                keypoints_ref_=keypoints_curr_;//特征点当前帧给了参考帧
                pts_3d_ref_=pt_curr_in_cameras;//更新这参考帧当中的特征点的位置
                keypoints_curr_.clear();
                //提取第一帧所有特征点的深度
            break;
            }
         case OK:
            /* code */
            {
                curr_=frame;
                keypoints_curr_.clear();
                orb_->detect(curr_->color_, keypoints_curr_);
                std::cout<<"第二次提取到特征点的数量"<<keypoints_curr_.size();
                std::cout<<"开始第二次提取特征点"<<std::endl;
                cv::Mat descriptors_curr_temp;
                descriptors_curr_=cv::Mat();
                orb_->compute(curr_->color_,keypoints_curr_,descriptors_curr_);//到这里肯定是没问题的
                //然后在这里进行特征的匹配
                cv::BFMatcher matchter_bf(cv::NORM_HAMMING2);
                vector<cv::DMatch> feature_matches_temp;
                //std::cout<<"参考帧:"<<descriptors_ref_<<std::endl;
                //std::cout<<"当前帧:"<<descriptors_curr_temp<<std::endl;
                // cv::imshow("curr",descriptors_curr_temp);
                // cv::waitKey(0);
                // cv::imshow("ref",descriptors_ref_);
                // cv::waitKey(0);
              //  if(descriptors_ref_==descriptors_curr_temp)
                // if(matIsEqual(descriptors_curr_temp,descriptors_ref_))//判断两个mat是否完全相同
                // {
                //     cout<<"现在的状态是当前帧和参考帧的描述子状态相同"<<endl;
                // }
               // matchter_bf.match(descriptors_ref_,descriptors_curr_,feature_matches_temp);//把当前帧和上一帧对匹配
                matchter_bf.match(descriptors_ref_,descriptors_curr_,feature_matches_temp);//把当前帧和上一帧对匹配
                std::cout<<"原始的特征点的数量:"<<feature_matches_temp.size()<<std::endl;
                //  for(int i=0;i<feature_matches_temp.size();i++)
                // {
                //     //对内部的数据进行遍历
                //     if(feature_matches_temp[i].distance<min_dist)
                //     min_dist=feature_matches_temp[i].distance;
                // }
                // std::cout<<"最小的距离"<<min_dist<<std::endl;
                //为什么这边的判断没有作用

                for(int i = 0 ; i < feature_matches_temp.size();i++)
                {
                    if(feature_matches_temp[i].distance<30)
                    {
                       // std::cout<<"进入条件特征的判断:"<<feature_matches_temp[i].distance<<std::endl;
                        feature_matches_.push_back(feature_matches_temp[i]);
                    }
                }
                 std::cout<<"good_match 的数量"<<feature_matches_.size()<<std::endl;
                  cv::Mat image_show;
                  //cv::drawMatches(ref_->color_,keypoints_ref_,curr_->color_,keypoints_curr_,feature_matches_,image_show);
                  image_show=ref_->color_;
                  cv::imshow("good",image_show);
                  cv::waitKey(1);
                   //特征匹配应该是对的
                //然后同样提取深度图,然后求解出pnp,求解出整个运动过程中相机的姿态
                //现在是将所有好的特征点都汇总进去 feature_matches_,
                //然后通过好的特征点进行求解相机的姿态
                //首先提取当前帧的深度图
                vector<cv::Point3d> pt_curr_in_cameras_in_good_match;
                vector<cv::Point2d>  pt_curr_in_pixels_in_good_match;
                //然后算出对应特征点的的空间位置
               // std::cout<<"参考帧::::"<<pts_3d_ref_<<std::endl;
                for(cv::DMatch m:feature_matches_)
                {
                    //将好的参考帧当中点弄进去
                  //  double image1_depth=ref_->depth_.ptr<ushort>(int(keypoints_ref_[m.queryIdx].pt.y))[int(keypoints_ref_[m.queryIdx].pt.x)];
                  //  if(image1_depth==0) continue;
                    pt_curr_in_cameras_in_good_match.push_back(pts_3d_ref_[m.queryIdx]);//将参考帧
                   // std::cout<<"空间当中特征点的坐标:"<<pts_3d_ref_[m.queryIdx]<<std::endl;
                    pt_curr_in_pixels_in_good_match.push_back(keypoints_curr_[m.trainIdx].pt);//将第二帧
                   // std::cout<<"对应的特征点的像素:"<<keypoints_curr_[m.trainIdx].pt<<std::endl;
                }
                std::cout<<"空间当中点的数量"<<pt_curr_in_cameras_in_good_match.size()<<"以及点"<<pt_curr_in_cameras_in_good_match[1]<<std::endl;
               // std::cout<<"像素当中的点:"<<pt_curr_in_pixels_in_good_match.size()<<"一级店"<<pt_curr_in_pixels_in_good_match[1]<<std::endl;
                //然后求解出pnp,恢复出相机的运动
                std::cout<<"开始求解相机运动"<<std::endl;
               // std::cout<<"第一个特征点的坐标:"<<pt_curr_in_pixels_in_good_match<<endl;
                //std::cout<<"fx:"<<ref_->camera_->cx_<<std::endl;
                cv::Mat K_3d_2d=(cv::Mat_<double>(3,3)<<517.3,0,325.1,0,516.5,249.7,0,0,1);
               std::cout<<K_3d_2d<<std::endl;

                cv::Mat R_curr2ref,t_curr2ref,R_curr2ref_temp;
                 //solvePnP这个 
                 cv::Mat  inliers;
                 //std::cout<<"对算法的输入"<<pt_curr_in_cameras_in_good_match<<std::endl;
                 //cv::solvePnP(pt_curr_in_cameras_in_good_match,pt_curr_in_pixels_in_good_match,K_3d_2d,cv::Mat(),R_curr2ref_temp,t_curr2ref,false);
                cv::solvePnPRansac(pt_curr_in_cameras_in_good_match,pt_curr_in_pixels_in_good_match,K_3d_2d,cv::Mat(),R_curr2ref_temp,t_curr2ref,false,100, 4.0, 0.99, inliers);//需要配置后面的参数,十分重要
                 std::cout<<"内点的数量:"<<inliers.rows<<std::endl;
                 if(inliers.rows<20)
                 {
                     std::cout<<"内点的数量太少"<<std::endl;
                 }
                 //这个内点数量肯定是有问题的
               //  cv::Rodrigues(R_curr2ref_temp,R_curr2ref);//转化成旋转矩阵的表示形式
                 // std::cout<<"求解相机运动完成"<<std::endl;

                  //无法正确求解出pnp
                  // std::cout<<"输出求解出的相机在相机坐标系下的平移距离"<<t_curr2ref<<endl;//求解出的这个有问题


                //求解出相机的姿态
                //然后通过ceresba进行呢优化
                ceres_ba(pt_curr_in_cameras_in_good_match,pt_curr_in_pixels_in_good_match,K_3d_2d,R_curr2ref_temp,t_curr2ref);
                //之后对相机的姿态进行累加
                T_c_r_estimated_= SE3(Sophus::SO3(R_curr2ref_temp.at<double>(0,0),R_curr2ref_temp.at<double>(1,0),R_curr2ref_temp.at<double>(2,0))
               ,Eigen::Vector3d(t_curr2ref.at<double>(0,0),t_curr2ref.at<double>(1,0),t_curr2ref.at<double>(2,0)));
               // std::cout<<"参考帧和当前帧之间的旋转关系:"<<T_c_r_estimated_.matrix()<<std::endl;
                curr_->T_c_w_=T_c_r_estimated_*ref_->T_c_w_;
                  //然后需要把当前帧的好的特征点给了下一帧
                  //这边提取深度是了下一帧考虑的
                  pts_3d_ref_.clear();//首先将原始数据进行清零
                  descriptors_ref_=cv::Mat();
                   vector<cv::Point3d>  pt_curr_in_cameras_2;
                   vector<cv::Point2d>  pt_curr_in_pixels_2;
                for(int i=0;i<keypoints_curr_.size();i++)
                {
                    double curr_depth_2=curr_->depth_.ptr<ushort>(int(keypoints_curr_[i].pt.y))[int(keypoints_curr_[i].pt.x)];
                    //计算特征点的像素和3d点
                    if(curr_depth_2==0) continue;
                     Eigen::Vector2d u_v_2(keypoints_curr_[i].pt.x,keypoints_curr_[i].pt.y);
                     cv::Point2d curr_u_v_2(u_v_2(0,0),u_v_2(1,0));
                     Eigen::Vector3d point_in_camera_2=curr_->camera_->pixel2camera(u_v_2,curr_depth_2);
                   // std::cout<<"point_in_camera:"<<point_in_camera_<<std::endl;
                   // Eigen::Vector3d point_in_camera_(1,1,1);
                    cv::Point3d Pt_in_camera_2(point_in_camera_2(0,0),point_in_camera_2(1,0),point_in_camera_2(2,0));
                    pt_curr_in_cameras_2.push_back(Pt_in_camera_2);
                    pt_curr_in_pixels_2.push_back(curr_u_v_2);
                    descriptors_ref_.push_back(descriptors_curr_.row(i));
                }
                //然后在通过恢复出的相机运动,再求解出空间当中特征点的位置
              //  descriptors_ref_=descriptors_curr_;//利用完特征之后,将特征进行变换
                pts_3d_ref_=pt_curr_in_cameras_2;//更新这参考帧当中的特征点的位置
                keypoints_ref_=keypoints_curr_;
                ref_=curr_;
                feature_matches_.clear();
                feature_matches_temp.clear();
                keypoints_curr_.clear();
            break;
            }
         case LOST:
         {
             break;
         }
        default:
            break;
        }
        return true;
    
    }   

   


};





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

