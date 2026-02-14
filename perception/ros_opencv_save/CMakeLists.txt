cmake_minimum_required(VERSION 2.8.3)
project(ros_opencv_save)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport message_generation std_msgs roscpp)

#SET(OpenCV_DIR /usr/local/lib/cmake/opencv4)
#SET(OpenCV_INCLUDE_DIRS "/usr/local/lib;/usr/local/lib;/usr/local/include/opencv4;/usr/local/lib/cmake/opencv4")
#SET(OpenCV_LIB_DIR "/usr/local/lib")

find_package(OpenCV REQUIRED)

add_message_files(FILES MsgTutorial.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  LIBRARIES ros_opencv_save
  CATKIN_DEPENDS std_msgs roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(ros_opencv_save src/ros_opencv_save.cpp)
add_dependencies(ros_opencv_save ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} msg_pkg_generate_messages_cpp)
target_link_libraries(ros_opencv_save ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})