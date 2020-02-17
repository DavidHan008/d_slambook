
#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "common_include.h"
#include "map.h"
#include <opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgcodecs.hpp>
#include<ceres/ceres.h>
#include<ceres/rotation.h>

#include <opencv2/features2d/features2d.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<sophus/so3.h>

namespace myslam 
{
class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOState {
        INITIALIZING=-1,
        OK=0,
        LOST
    };
    
    VOState     state_;     // current VO status 当前帧的状态
    Map::Ptr    map_;       // map with all frames and map points
    Frame::Ptr  ref_;       // reference frame  参考帧
    Frame::Ptr  curr_;      // current frame  当前帧
    
    cv::Ptr<cv::ORB> orb_;  // orb detector and computer 
    vector<cv::Point3d>     pts_3d_ref_;        // 3d points in reference frame  参考帧当中的点 上一帧估计的姿态
    vector<cv::KeyPoint>    keypoints_curr_;    // keypoints in current frame 当前帧的关键点
    vector<cv::KeyPoint>    keypoints_ref_;    // keypoints in current frame 当前帧的关键点
    Mat                     descriptors_curr_;  // descriptor in current frame 
    Mat                     descriptors_ref_;   // descriptor in reference frame 
    vector<cv::DMatch>      feature_matches_;//特征匹配
    
    SE3 T_c_r_estimated_;  // the estimated pose of current frame 上一帧(参考帧) 
    int num_inliers_;        // number of inlier features in icp
    int num_lost_;           // number of lost times
    
    // parameters 
    int num_of_features_;   // number of features
    double scale_factor_;   // scale in image pyramid 图像金字塔的只读
    int level_pyramid_;     // number of pyramid levels 层数
    float match_ratio_;      // ratio for selecting  good matches 好的点的哥说
    int max_num_lost_;      // max number of continuous lost times
    int min_inliers_;       // minimum inliers 其实这些参数没什么意义
    
    double key_frame_min_rot;   // minimal rotation of two key-frames
    double key_frame_min_trans; // minimal translation of two key-frames
    double min_dist;
       int num=1;
    
public: // functions 
    VisualOdometry();
    ~VisualOdometry();
    
    bool addFrame( Frame::Ptr frame );      // add a new frame  是否添加新的帧
    
protected:  
    // inner operation 
    void extractKeyPoints();
    void computeDescriptors(); 
    void featureMatching();
    void poseEstimationPnP(); 
    void setRef3DPoints();
    
    void addKeyFrame();
    bool checkEstimatedPose(); 
    bool checkKeyFrame();
    
};
}

#endif // VISUALODOMETRY_H