cmake_minimum_required(VERSION 3.0.2)
project(video_stream)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  #image_transport
)

#set(OpenCV_DIR /usr/local/lib/cmake/opencv4/OpenCVConfig.cmake)

find_package(OpenCV 4.6.0 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  #/usr/local/include
  /usr/local/include/opencv4
  #include
)



add_executable(
  webcam_stream
  src/webcam_stream.cc
)

target_link_libraries(webcam_stream ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(
  webcam_stream 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# catkin_install_python(PROGRAMS scripts/webcam_stream.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )