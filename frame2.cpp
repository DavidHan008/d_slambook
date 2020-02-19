#include"frame2.h"
namespace myslam
{
    Frame::Ptr  Frame::CreateFrame()
    {
        static unsigned long factory_id=0;//尤其是这个id的用法
        Frame::Ptr curr_frame(new Frame);
        curr_frame->id_=factory_id;
        factory_id++;
        return curr_frame;
    }
}