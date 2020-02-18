#include<iostream>
#include"visual_odometry2.h"
int main(int argc, const char** argv) {
    //第一步,实例化一个vo的指针
     myslam::VisualOdometry::Ptr vo_ptr ( new myslam::VisualOdometry);
     assert(vo_ptr->Init()==true);
     vo_ptr->Run();
    return 0;
}