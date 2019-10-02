cmake_minimum_required( VERSION 2.8 )
project( RGBD )

set(CMAKE_CXX_FLAGS "-std=c++11")

# 添加Eigen头文件

find_package( Eigen3 3.3 REQUIRED NO_MODULE)
#target_link_libraries( visualizeGeometry Eigen3::Eigen) 

# 添加Pangolin依赖
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

find_package( OpenCV REQUIRED)

add_executable( RGBD jointMap.cpp )
target_link_libraries( RGBD ${Pangolin_LIBRARIES} )
target_link_libraries( RGBD ${OpenCV_LIBS} )
