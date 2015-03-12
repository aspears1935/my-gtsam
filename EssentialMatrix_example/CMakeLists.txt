cmake_minimum_required(VERSION 2.8)
project( GTSAMTest )
find_package( OpenCV REQUIRED )

include_directories(/home/temp/gtsam-3.2.0)
include_directories(/home/temp/gtsam-3.2.0/build)
include_directories(/home/temp/gtsam-3.2.0/build/gtsam)
link_directories(/home/temp/gtsam-3.2.0/build)
link_directories(/home/temp/gtsam-3.2.0/build/gtsam)

add_executable( GTSAMTest EssentialMatrixExample.cpp )
target_link_libraries( GTSAMTest gtsam boost_system tbb tbbmalloc)
