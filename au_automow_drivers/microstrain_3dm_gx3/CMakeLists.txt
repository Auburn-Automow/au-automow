cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_gensrv()
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(imu_node src/imu_node.cc)
target_link_libraries(imu_node 3dmgx3)
rosbuild_add_library(3dmgx3 src/3dmgx3.cc)

# Utility to recover ID from IMU
rosbuild_add_executable(get_id src/get_id.cpp)
target_link_libraries(get_id 3dmgx3)

# Utility to publish TF from imu to ground (no heading)
rosbuild_add_executable(ground_tf src/ground_tf.cc)