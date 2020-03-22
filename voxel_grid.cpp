#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include<opencv2/core/eigen.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>	
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

int main(int argc, const char** argv) {
    cv::Mat source_image=cv::imread(argv[1]);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(argv[2], *cloud_in) == -1) //* load the ply file 
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        system("PAUSE");
        return (-1);
    }
    //载入相机外参,这个外参相机坐标系下的激光雷达的位移+旋转
    //针对这个旋转,相当于吧激光雷达旋转到相机的四元数
    
    cv::imshow("out_image",out_image);
    // cv::imwrite("outimage.jpg",out_image);
    cv::waitKey(0);
    return 0;
}