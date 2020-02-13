#include<iostream>
#include<ceres/ceres.h>
#include <random>//用于产生随机数

//这个例子求解的是x^2+2x+1=0  在某个地方的最小值
//这是单一变量,  
struct d_CostFunctor
{
 template <typename T>
 bool operator()(const T* const x,T* residual)const{
     //现在想求的是x^2+2x+1=0 使得他最小的数值 x[0]表示只有需要求的量
    //  residual[0] = x[0]*x[0]+2*x[0]+1;//residual 表示剩余量 这边是能写出他的导数就是写成 residual[0]=xxx-xx=0的形式
    residual[0] = x[0]*x[0]+x[0]+x[0]+1.0;//必须有小数点
     return true;
 }
};
 //针对与多个变量,我希望y-x^2-2x-1=0 这个方程最小
 struct d_CostFunctor_2
{
 template <typename T>
 bool operator()(const T* const xy,T* residual)const{
    residual[0] = xy[1]-xy[0]*xy[0]-xy[0]-xy[0]-1.0;//必须有小数点
    //residual[1],表示他的一阶导数,以此类推
     return true;
 }
};

//针对于一条曲线 y=exp(3x^2+2x+1),其中,你的ax^2+bx+c你有三个量你是不知道的
 struct d_curve
 {
     /* data */
     //那么我需要输入的x,y的data
     d_curve(double x,double y):_x(x),_y(y){}
     template <typename T>
     bool operator()(const T* const abc,T* r)const{
        r[0]=_y-ceres::exp(abc[0]*_x*_x+abc[1]*_x+abc[2]);//使得某个
        return true;
     }
    const double _x,_y;
 };

int main( int argc, char** argv )
{
    //第一部分 自动求导
    //针对单一变量的
    //第一步初始化参数
    double curr_x=100;
   //第二步构建这些问题
   ceres::Problem d_pro;
   //第一个参数::::::第一个1是输出维度，即残差的维度，第二个1是输入维度，即待寻优参数x的维度。
   ceres::CostFunction* d_cost_f=new ceres::AutoDiffCostFunction<d_CostFunctor,1,1>(new d_CostFunctor);
   d_pro.AddResidualBlock(d_cost_f,NULL,&curr_x) ; //单个一个数字是加&的 
   //运行这个求解器
    ceres::Solver::Options d_opt;
    d_opt.linear_solver_type=ceres::DENSE_QR;
    d_opt.minimizer_progress_to_stdout=true;
    ceres::Solver::Summary sum;
    ceres::Solve(d_opt,&d_pro,&sum); 
    std::cout << sum.BriefReport() << "\n";//输出优化的简要信息
    std::cout<<"x:"<<curr_x<<std::endl;
   //针对两个变量的时候
     double curr_xy_2[2]={1,1};//xy 设定初始数值
     ceres::Problem d_ptr_2;
     //因为输出的xy 两个参数
     //由于输出的仍然是一个这个数组,所以是一维,但是这个数组里面是有两个参数的,因此后面的第二个参数是2
     ceres::CostFunction* d_cost_f_2=new ceres::AutoDiffCostFunction<d_CostFunctor_2,1,2>(new d_CostFunctor_2);
     d_ptr_2.AddResidualBlock(d_cost_f_2,nullptr,curr_xy_2);//如果是一个数组,直接放数组名称,一本本身就是一个指针
     ceres::Solver::Options d_opt2;
     d_opt2.linear_solver_type=ceres::DENSE_QR;
     d_opt2.minimizer_progress_to_stdout=true;
     ceres::Solver::Summary sum2;
     ceres::Solve(d_opt2,&d_ptr_2,&sum2);
     std::cout<<sum2.BriefReport()<<std::endl;
     std::cout<<"curr_x_2:"<<curr_xy_2[0]<<";"<<"curr_y_2:"<<curr_xy_2[1]<<std::endl;

        //第二部 曲线拟合
        //对去曲线拟合,那么我输入的是一些数据,然后我希望通过这些数据,拟合出一条曲线
        //得到时候曲线的参数的相关的系数
        double w_sigma=1.0;                 // 噪声Sigma值
         std::default_random_engine e;
	     std::uniform_real_distribution<double> u(-1.2,1.5);//随机数字的范围
         std::vector<double> x_data,y_data;
         double y=0;
             std::cout.precision(8); //设置小数点个数
         //数据产生好了
         for(double x=0;x<1000;x++)//这边必须是double
         {
             double xx=x/1000;//设定数据最大就是1
             y=std::exp(3*xx*xx+2*xx+1) +w_sigma*u(e); 
             x_data.push_back(xx);
             y_data.push_back(y);
         }
         //构建求解器
         //其实说白了就是在实例化的时候把相关的参数传递进去
         double abc[3]={0,0,0};//求解变量的初值
         ceres::Problem d_curve_ptr;
         for(int i=0;i<x_data.size();i++)
         {
            ceres::CostFunction *d_curve_cost=new ceres::AutoDiffCostFunction<d_curve,1,3>(new d_curve(x_data[i],y_data[i]));
         //   d_curve_ptr.AddResidualBlock(d_curve_cost,nullptr,abc);  
              d_curve_ptr.AddResidualBlock(d_curve_cost,new ceres::CauchyLoss(0.5),abc);  //利用鲁棒核函数进行求解
         };
         ceres::Solver::Options d_curve_opt;
         d_curve_opt.linear_solver_type=ceres::DENSE_QR;
         d_curve_opt.minimizer_progress_to_stdout=true;
         ceres::Solver::Summary d_curve_sum;
         ceres::Solve(d_curve_opt,&d_curve_ptr,&d_curve_sum);
         std::cout<<d_curve_sum.BriefReport()<<std::endl;
         std::cout<<"a:"<<abc[0]<<";   b:"<<abc[1]<<";    c"<<abc[2]<<std::endl;
    return 0;
}