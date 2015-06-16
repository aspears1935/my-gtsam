cmake_minimum_required(VERSION 2.8)
project( CamSonarFusion )
find_package( OpenCV REQUIRED )

include_directories(/home/temp/gtsam-3.2.0)
include_directories(/home/temp/gtsam-3.2.0/build)
include_directories(/home/temp/gtsam-3.2.0/build/gtsam)
link_directories(/home/temp/gtsam-3.2.0/build)
link_directories(/home/temp/gtsam-3.2.0/build/gtsam)

add_executable( CamSonarFusion cam_sonar_fusion.cpp )
target_link_libraries( CamSonarFusion gtsam boost_system tbb tbbmalloc)
