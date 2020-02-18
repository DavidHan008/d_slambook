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
        // frontend_=new(myslam::Frontend);
        // backend_=new(myslam::Backend);
        return true;
    }
    void VisualOdometry::Run()
    {
        while (1)
        {
            //循环提取两帧图像
            dataset_->NextFrame();
            /* code */

        }
        
    }
    bool VisualOdometry::Step()
    {

    }







}