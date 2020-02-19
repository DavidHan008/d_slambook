#include"frontend2.h"
#include"feature2.h"
namespace myslam
{
    Frontend::Frontend()
    {
        //初始化检测算子
        gftt_=cv::GFTTDetector::create(150,0.01, 20);
        //后面用orb来测试一下
    }
    bool Frontend::AddFrame(Frame::Ptr frame)
    {
        current_frame_=frame;//之后使用函数,就可以用类内的成员函数进行操作
        switch (status_)
        {
        case  FrontendStatus::INITING:
            assert(StereoInit()==true);
            //然后对第一张图片评估姿态
            break;
        case FrontendStatus::TRACKING_GOOD:
            assert(Track()==true);
            break;
         case FrontendStatus::TRACKING_BAD:
            assert(Reset()==true);
            status_=FrontendStatus::INITING;
            break;
         case  FrontendStatus::LOST:
            /* code */
            break;
        
        default:
            break;
        }
        last_frame_=frame;
        return true;
    }
    bool Frontend::Track()
    {
        return true;
    }
    bool Frontend::Reset()
    {
        return true;
    }
    // int Frontend::TrackLastFrame()
    // {
    //     return 1;
    // }
    int Frontend::EstimateCurrentPose()
    {
        return 1;
    }
    // bool Frontend::InsertKeyframe()
    // {
    //     return true;
    // }
    bool Frontend::StereoInit()
    {
        int feature_num=DetectFeatures();
        return true;
    }
    int Frontend::DetectFeatures()
    {
        std::vector<cv::KeyPoint> keypoints;
        gftt_->detect(current_frame_->left_img_,keypoints);
        int kpt_nums=0;
        for(cv::KeyPoint kp:keypoints)
        {
            kpt_nums++;
            //Feature::Ptr featurePtr=new Feature::Ptr(Feature(current_frame_,kp));
            //current_frame_->features_left_.push_back(featurePtr);
        }
        return kpt_nums;
    }
    int Frontend::FindFeaturesInRight()
    {
        std::vector<uchar> status;
        Mat error;
        cv::calcOpticalFlowPyrLK();

        return 1;
    }
    int Frontend::TriangulateNewPoints()
    {
        return 1;
    }
    // bool Frontend::BuildInitMap();
    // {
    //     return true;
    // }
    // void Frontend::SetObservationsForKeyFrame()
    // {
    //     return 1;
    // }




}