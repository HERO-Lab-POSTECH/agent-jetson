cmake_minimum_required(VERSION 2.8.3)
project(ros_opencv_manipulation)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  message_generation
)
add_message_files(FILES hero_manipulation_msg.msg)
find_package(OpenCV REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
)

catkin_package(
  LIBRARIES ros_opencv_manipulation
  CATKIN_DEPENDS std_msgs roscpp
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
   )


add_executable(${PROJECT_NAME} src/ros_opencv_manipulation.cpp)
add_dependencies(ros_opencv_manipulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} msg_pkg_generate_messages_cpp)
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
