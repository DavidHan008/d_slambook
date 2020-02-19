#include"dataset2.h"
#include"frame2.h"
#include<fstream>
namespace myslam{
    Dataset::Dataset(const std::string & dataset_path)
    {
        dataset_path_=dataset_path;
        //assert(init()==true);
    }

    bool Dataset::Init()
    {
        //提取time.txt然后统计出来,有多少行,那么就说明有多少个
          std::fstream ass_file_;
          ass_file_.open(dataset_path_+"times.txt");
          std::vector<double> times;
          double time;
            while(!ass_file_.eof())
            {
                ass_file_>>time;
                times.push_back(time);
            }
            ass_file_.close();
        std::cout<<"总共的图片数量:"<<times.size()<<std::endl;
        return true;
    }
    Frame::Ptr Dataset::NextFrame()
    {
        //实例化一个工厂函数
        Frame::Ptr frame_curr= Frame::CreateFrame();//这个工厂函数值得学习的地方
        std::stringstream ss;
        ss<<std::setfill('0')<< std::setw(6)<<frame_curr->id_<<".png";//setw设置有总共的数据长度,不够的弥补0
        //然后进行填充成一个6位置,数字
        //这里返回的是Frame的id
        //如何使用计数的方式来提取
        std::string LeftImageFilePath,RightImageFilePath;
        LeftImageFilePath=dataset_path_+"image_0/"+ss.str();
        RightImageFilePath=dataset_path_+"image_1/"+ss.str();
        std::cout<<"image_name:"<<LeftImageFilePath<<std::endl;
        frame_curr->left_img_=cv::imread(LeftImageFilePath);
        frame_curr->right_img_=cv::imread(RightImageFilePath);
        // cv::imshow("image",frame_curr->left_img_);
        // cv::waitKey(0);
        return frame_curr;
    }
}