#include"frame2.h"
namespace myslam
{
    Frame::Ptr  Frame::CreateFrame()
    {
        static unsigned long factory_id=0;
        Frame::Ptr curr_frame(new Frame);
        curr_frame->id_=factory_id;
        factory_id++;
        return curr_frame;
    }
}