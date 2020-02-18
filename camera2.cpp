#include"camera2.h"
namespace myslam {
    Camera::Camera()
    {
        //         camera.fx: 517.3
        // camera.fy: 516.5
        // camera.cx: 325.1
        // camera.cy: 249.7
        fx_=517.3;
        fy_=516.5;
        cx_=325.1;
        cy_=249.7;
        baseline_=1;
    }
     Vec3 Camera::world2camera( const Vec3& p_w, const SE3& T_c_w )
      {
          //首先第一个,从世界坐标系,到归一化坐标系,这边输入的旋转矩阵
          Vec3 point_in_camera;
          point_in_camera=T_c_w*p_w;//左乘以变换矩阵
          //相机坐标系的点的坐标=从世界坐标系到相机坐标系的变换矩阵*这个点在世界坐标系下的坐标
          return T_c_w*p_w;
      }
    Vec3 Camera::camera2world( const Vec3& p_c, const SE3& T_c_w )
     {
          Vec3 point_in_word;
          point_in_word=T_c_w.inverse()*p_c;//左乘以变换矩阵
         //SE3是一个4*4的矩阵
          return point_in_word;
     }
       Vec2 Camera::camera2pixel( const Vec3& p_c )
       {
           //已经知道一个点在相机坐标系下的坐标,然后求解出u 和v 
           Vec2 u_v;
           u_v[0]=p_c[0]*fx_/p_c[2]+cx_;//u
           u_v[1]=p_c[1]*fy_/p_c[2]+cy_;//v
           return u_v;
       }
        Vec3 Camera::pixel2camera( const Vec2& p_p, double depth )
        {
            Vec3 point_in_camera;
            //std::cout<<"depth_scale_:"<<depth_scale_<<std::endl;
            point_in_camera[0]=(p_p[0]-cx_)*depth/fx_;
            point_in_camera[1]=(p_p[1]-cy_)*depth/fy_;
            point_in_camera[2]=depth;
            return point_in_camera;
        }
        Vec3 Camera::pixel2world ( const Vec2& p_p, const SE3& T_c_w, double depth )
        {
            //首先从像素坐标到相机坐标,然后从相机坐标到世界坐标
            Vec3 point_in_camera;
            Vec2 p=p_p;
            point_in_camera=pixel2camera(p,depth);
            Vec3 point_in_word;
            point_in_word=camera2world(point_in_camera,T_c_w);
            return point_in_word;
        }

         Vec2 Camera::world2pixel ( const Vec3& p_w, const SE3& T_c_w )
         {
             Vec3 point_in_camera;
             point_in_camera=world2camera(p_w,T_c_w);
             Vec2 point_in_pixel=camera2pixel(point_in_camera);
             return point_in_pixel;
         }
}