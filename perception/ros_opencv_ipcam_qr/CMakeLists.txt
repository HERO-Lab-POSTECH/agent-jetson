cmake_minimum_required(VERSION 2.8.3)
project(ros_opencv_ipcam_qr)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport message_generation std_msgs roscpp)

add_message_files(FILES hero_ipcam_qr_msg.msg)
find_package(OpenCV REQUIRED)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  LIBRARIES ros_opencv_ipcam_qr
  CATKIN_DEPENDS std_msgs roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(ros_opencv_ipcam_qr src/ros_opencv_ipcam_qr.cpp)
add_dependencies(ros_opencv_ipcam_qr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} msg_pkg_generate_messages_cpp)
target_link_libraries(ros_opencv_ipcam_qr ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})