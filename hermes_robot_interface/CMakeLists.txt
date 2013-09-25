cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# these 3 lines activate automatic action header file generation if you have specified actions in the /action folder
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ros/include
		    ${PROJECT_SOURCE_DIR}/common/include)


rosbuild_add_library(m5api common/src/Device/Device.cpp  common/src/Device/ProtocolDevice.cpp   common/src/Device/PCanDevice.cpp common/src/Device/ProtocolMessage.cpp  common/src/Device/RS232Device.cpp  common/src/Util/Random.cpp  common/src/Util/Message.cpp common/src/Util/StopWatch.cpp common/src/Util/IOFunctions.cpp  common/src/Util/Thread.cpp  common/src/M5apiw32/m5apiw32.cpp)

rosbuild_add_compile_flags(m5api -D__LINUX__ -DUSE_PCAN )


#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(hermes_robot_interface_node ros/src/hermes_robot_interface_main.cpp
													ros/src/hermes_robot_interface.cpp
										            common/src/hermes_interface/hermesInterface.cpp)
target_link_libraries(hermes_robot_interface_node m5api  pcan moveit_move_group_interface)


