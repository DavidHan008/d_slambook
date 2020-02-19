#include"visual_odometry2.h"
#include"dataset2.h"
namespace myslam
{
    VisualOdometry::VisualOdometry()
    {

    }
    bool VisualOdometry::Init()
    {
        //初始化frame, map,dataset,
        std::string filePath="/home/davidhan/davidhan_project/kitti/00/";
        dataset_=myslam::Dataset::Ptr(new myslam::Dataset(filePath));
        assert(dataset_->Init()==true);
        // viewer_=new(myslam::Viewer);
        frontend_=myslam::Frontend::Ptr(new myslam::Frontend);
        backend_=myslam::Backend::Ptr(new myslam::Backend);
        map_=myslam::Map::Ptr(new myslam::Map);
        return true;
    }
    void VisualOdometry::Run()
    {
        while (1)
        {
            //循环提取两帧图像
            Frame::Ptr frame_curr=dataset_->NextFrame();
            frontend_->AddFrame(frame_curr);
            /* code */

        }
        
    }
   







}