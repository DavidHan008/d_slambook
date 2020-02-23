# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -ggdb")

# 声明一个 cmake 工程
project( HelloSLAM )
include_directories("/usr/local/include/eigen3")
find_package( OpenCV 3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
#find_package(Ceres REQUIRED PATHS "/some/where/local/")

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )



set(Ceres_LIBS_DIR /usr/local/ceres/lib)
set(Ceres_DIR ${Ceres_LIBS_DIR}/cmake/Ceres)
find_package(Ceres REQUIRED)
message(STATUS "Found Ceres: ${CERES_FOUND}")
message(STATUS "    Ceres Include Dirs: ${CERES_INCLUDE_DIRS}")
message(STATUS "    Ceres Libs: ${CERES_LIBRARIES}")

include_directories(${CERES_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  geometry_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
# 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )

# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
add_executable( helloSLAM helloSLAM.cpp )

# 添加一个库
add_library( hello libHelloSLAM.cpp )
# 共享库
add_library( hello_shared SHARED libHelloSLAM.cpp )

add_executable( useHello useHello.cpp )
# 将库文件链接到可执行程序上
target_link_libraries( useHello hello_shared )

add_executable(d_cv_loop cv_loop.cpp )
target_link_libraries( d_cv_loop ${OpenCV_LIBS}  )




add_executable(d_joint_map joint_map.cpp )
target_link_libraries( d_joint_map ${OpenCV_LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable( d_use_ceres use_ceres.cpp )

set(LIBS ${Ceres_LIBS_DIR}/libceres.a umfpack cxsparse glog gflags gomp
        ccolamd btf klu cholmod lapack blas camd amd pthread)
target_link_libraries( d_use_ceres ${LIBS}  )

add_executable(d_vo d_vo_main.cpp
                    camera.cpp
                    visual_odometry.cpp
                    frame.cpp



 )
target_link_libraries( d_vo ${OpenCV_LIBS} ${catkin_LIBRARIES} ${Sophus_LIBRARIES} ${LIBS})


add_executable(d_cv_pose_trans cv_pose_trans.cpp )
target_link_libraries( d_cv_pose_trans ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${LIBS}  )

add_executable(d_ceres_map ceres_map.cpp )
target_link_libraries( d_ceres_map ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${LIBS}  )

add_executable(d_lk_dirtect lk_dirtect.cpp )
target_link_libraries( d_lk_dirtect ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${LIBS}  )


#add_executable(d_slam 
#                    camera2.cpp
#                    d_slam_main.cpp
#                    frame2.cpp
#                    dataset2.cpp
#                    visual_odometry2.cpp
#                    map2.cpp
#                    frontend2.cpp
#                    backend2.cpp
#
# )
#target_link_libraries( d_slam ${OpenCV_LIBS} ${catkin_LIBRARIES} ${Sophus_LIBRARIES} ${LIBS} ${Pangolin_LIBRARIES} )
