#include"camera.h"
namespace myslam
{
    Camera::Camera()
{
    fx_ = 1;
    fy_ =1;
    cx_ =1;
    cy_ = 1;
    depth_scale_ = 1;
}

     Vector3d Camera::world2camera( const Vector3d& p_w, const SE3& T_c_w )
      {
          //首先第一个,从世界坐标系,到归一化坐标系,这边输入的旋转矩阵
          Eigen::Vector3d point_in_camera;
          point_in_camera=T_c_w*p_w;//左乘以变换矩阵
          //相机坐标系的点的坐标=从世界坐标系到相机坐标系的变换矩阵*这个点在世界坐标系下的坐标
         //SE3是一个4*3的矩阵
          return T_c_w*p_w;
      }
     Vector3d Camera::camera2world( const Vector3d& p_c, const SE3& T_c_w )
     {
          Eigen::Vector3d point_in_word;
          point_in_word=T_c_w.inverse()*p_c;//左乘以变换矩阵
         //SE3是一个4*4的矩阵
          return point_in_word;
     }
       Vector2d Camera::camera2pixel( const Vector3d& p_c )
       {
           //已经知道一个点在相机坐标系下的坐标,然后求解出u 和v 
           Vector2d u_v;
           u_v[0]=p_c[0]*fx_/p_c[2]+cx_;//u
           u_v[1]=p_c[1]*fy_/p_c[2]+cy_;//v
           return u_v;
       }
        Vector3d Camera::pixel2camera( const Vector2d& p_p, double depth )
        {
            Vector3d point_in_camera;
            //std::cout<<"depth_scale_:"<<depth_scale_<<std::endl;
            point_in_camera[0]=(p_p[0]-cx_)*depth/depth_scale_/fx_;
            point_in_camera[1]=(p_p[1]-cy_)*depth/depth_scale_/fy_;
            point_in_camera[2]=depth/depth_scale_;
            return point_in_camera;
        }
        Vector3d Camera::pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth )
        {
            //首先从像素坐标到相机坐标,然后从相机坐标到世界坐标
            Vector3d point_in_camera;
            Vector2d p=p_p;
            point_in_camera=pixel2camera(p,depth);
            Vector3d point_in_word;
            point_in_word=camera2world(point_in_camera,T_c_w);
            return point_in_word;
        }

         Vector2d Camera::world2pixel ( const Vector3d& p_w, const SE3& T_c_w )
         {
             Vector3d point_in_camera;
             point_in_camera=world2camera(p_w,T_c_w);
             Vector2d point_in_pixel;
             return point_in_pixel;
         }
};