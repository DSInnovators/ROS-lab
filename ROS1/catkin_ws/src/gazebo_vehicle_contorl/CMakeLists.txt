cmake_minimum_required(VERSION 3.0.2)
project(gazebo_vehicle_contorl)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp  
  gazebo_msgs 
  geometry_msgs   
  gazebo_ros 
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    roscpp    
    geometry_msgs
    gazebo_msgs
    gazebo_ros 
)

#add_library(${PROJECT_NAME} SHARED src/model_push.cc)
#target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})
