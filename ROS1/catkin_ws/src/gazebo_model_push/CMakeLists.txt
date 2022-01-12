cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)
project(gazebo_model_push)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp  
  gazebo_msgs 
  geometry_msgs   
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
)

add_library(${PROJECT_NAME} SHARED src/model_push.cc)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})




