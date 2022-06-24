cmake_minimum_required(VERSION 3.0.2)
project(object_avoidance)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  roslib
  cv_bridge
)

set(OpenCV_DIR /usr/local/lib/cmake/opencv4/OpenCVConfig.cmake)

find_package(OpenCV 4.6.0 REQUIRED)

#message(STATUS "OpenCV library status:")
#message(STATUS "    version: ${OpenCV_VERSION}")
#message(STATUS "    libraries: ${OpenCV_LIBS}")
#message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

message(STATUS "CATKIN_PACKAGE_BIN_DESTINATION:       ${CATKIN_PACKAGE_BIN_DESTINATION}")
message(STATUS "CATKIN_PACKAGE_INCLUDE_DESTINATION:   ${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
message(STATUS "catkin_INCLUDE_DIRS:                  ${catkin_INCLUDE_DIRS}")
message(STATUS "catkin_EXPORTED_TARGETS:              ${catkin_EXPORTED_TARGETS}")
message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}: ${${CMAKE_CURRENT_SOURCE_DIR}}")
message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}: ${${CATKIN_DEVEL_PREFIX}}")


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
)

# file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/src/files
#   DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(
  object_detection
  src/object_detection.cc
)

target_link_libraries(object_detection ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(
  object_detection 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  object_avoidance
  src/object_avoidance.cc
)

target_link_libraries(object_avoidance ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(
  object_avoidance 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# catkin_install_python(PROGRAMS scripts/object_detection.py scripts/object_avoiding.py scripts/video_stream.py scripts/util.py scripts/coco.names
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )