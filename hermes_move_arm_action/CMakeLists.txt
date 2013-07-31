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

#uncomment if you have defined messages (also activate if you have defined own action messages!)
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

rosbuild_add_library(m5api common/src/Device/Device.cpp  common/src/Device/ProtocolDevice.cpp  common/src/Device/ESDDevice.cpp  common/src/Device/PCanDevice.cpp common/src/Device/ProtocolMessage.cpp  common/src/Device/RS232Device.cpp  common/src/Util/Random.cpp  common/src/Util/Message.cpp common/src/Util/StopWatch.cpp common/src/Util/IOFunctions.cpp  common/src/Util/Thread.cpp  common/src/M5apiw32/m5apiw32.cpp)

rosbuild_add_compile_flags(m5api -D__LINUX__ -DUSE_PCAN -DUSE_ESD)



# add build commands for action server
rosbuild_add_executable(move_arm_action ros/src/hermes_move_arm_action_server_main.cpp
					common/src/hermes_interface/hermesInterface.cpp
                                        ros/src/hermes_move_arm_action_server.cpp)

# add build commands for action client
rosbuild_add_executable(hermes_move_arm_action_client ros/src/hermes_move_arm_action_client_main.cpp
                                              ros/src/hermes_move_arm_action_client.cpp)


#samples
rosbuild_add_executable(leerModulosConectados common/src/samples/leerModulosConectados.cpp)
target_link_libraries(leerModulosConectados m5api)
target_link_libraries(move_arm_action m5api)

rosbuild_add_executable(moveManually_client common/src/samples/moveManually_client.cpp)
target_link_libraries(moveManually_client m5api)

