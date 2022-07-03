cmake_minimum_required(VERSION 3.0.2)
project(lidar_tutorial)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  sensor_msgs
  tf
)

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


catkin_package(
  INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES lidar_tutorial gazebo_ros_velodyne_laser gazebo_ros_velodyne_gpu_laser
  CATKIN_DEPENDS gazebo_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)


link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_executable(lidar_visualizer src/lidar_visualizer.cc)
add_dependencies(lidar_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_visualizer ${catkin_LIBRARIES})


add_library(gazebo_ros_velodyne_laser src/GazeboRosVelodyneLaser.cpp)
target_link_libraries(gazebo_ros_velodyne_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin
)

add_library(gazebo_ros_velodyne_gpu_laser src/GazeboRosVelodyneLaser.cpp)
target_link_libraries(gazebo_ros_velodyne_gpu_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  GpuRayPlugin
)
target_compile_definitions(gazebo_ros_velodyne_gpu_laser PRIVATE GAZEBO_GPU_RAY=1)

install(TARGETS gazebo_ros_velodyne_laser gazebo_ros_velodyne_gpu_laser
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)