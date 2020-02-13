#include"frame.h"
 namespace myslam
{
    Frame::Frame()
    {
        id_=-1;
        time_stamp_=0;
        camera_=nullptr;
    }
    Frame::~Frame()
    {

    }
};
