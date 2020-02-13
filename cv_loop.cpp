#include <iostream>
#include <opencv2/core.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>

int main( int argc, char** argv )
{
       //提取一张图片,然后遍历这个图片上的所有的数据,存储下来,然后吧这个数据都变成0;
    //@1读取图片
    cv::Mat source_image, gray_image;
    source_image=cv::imread(argv[1]);
    //这张图片的数据应该是在0-255时间,那么都用uint8
    //灰度化
    cv::cvtColor(source_image, gray_image, cv::COLOR_RGB2GRAY);
    std::vector<uchar>gray_images_data;
    //对图像的长和宽大小进行遍历
    //对于一个mat数据类型,我是知道长和宽的
    std::cout<<"cols     x宽列:"<<source_image.cols<<std::endl;
    std::cout<<"rows    y行高:"<<source_image.rows<<std::endl;
    for(int y=0;y<source_image.cols;y++)//y
    {
        for(int x=0;x<source_image.rows;x++)
        {
            //图像在某一点的像素数值
            //images_uv=source_image[k][j];
            //使用mat类型的指针
            //所有opencv里面的数据都是uchar类型
            uchar image_data=gray_image.at<uchar>(x,y);
            //我可以对这些进行赋值
            gray_images_data.push_back(image_data);
        }
    }
    cv::imshow("231",gray_image);
    cv::waitKey(0);
    return 0;
}
