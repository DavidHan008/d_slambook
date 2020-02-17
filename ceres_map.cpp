#include<iostream>
#include<fstream>
void read_data(const char**argv);

 int main(int argc, const char** argv) {
     read_data(argv);
    std::cout<<"输入的data"<<argv[1]<<std::endl;
    return 0;
}
void read_data(const char**argv)
{
 std::fstream ass_file_; 
 std::string filename=argv[0];
 ass_file_.open(filename);//为了方便写的绝对路径
 int camera_num,kp_num,observice_num;
 ass_file_>>camera_num>>kp_num>>observice_num;
 //意思是16个相机,xx个特征点,共同组成了xx个观测数据
}