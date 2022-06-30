cmake_minimum_required(VERSION 3.0.2)
project(auto_vehicle)

## Find catkin and any catkin packages
find_package(catkin REQUIRED 
  COMPONENTS 
  roscpp 
  auto_vehicle_msgs
  rospy 
  std_msgs 
  sensor_msgs 
  genmsg   
  cv_bridge
  image_transport
  )
find_package(OpenCV 4.2.0 REQUIRED)


# ## Declare ROS messages and services
# add_message_files(DIRECTORY msg FILES Num.msg)
# add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

## Declare a catkin package
catkin_package(INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS auto_vehicle_msgs roscpp
   DEPENDS eigen opencv)

## Build talker and listener
include_directories(
  include ${catkin_INCLUDE_DIRS}
  /usr/include/opencv4/
)

add_executable(automate_vehicle 
  src/automate_vehicle.cc
  src/RoadDetector.cc)
target_link_libraries(automate_vehicle ${catkin_LIBRARIES})
add_dependencies(automate_vehicle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# add_dependencies(automate_vehicle auto_vehicle_generate_messages_cpp)

# add_executable(listener src/listener.cpp)
# target_link_libraries(listener ${catkin_LIBRARIES})
# add_dependencies(listener beginner_tutorials_generate_messages_cpp)

# add_executable(add_two_ints_server src/add_two_ints_server.cpp)
# target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
# add_dependencies(add_two_ints_server beginner_tutorials_gencpp)

# add_executable(add_two_ints_client src/add_two_ints_client.cpp)
# target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
# add_dependencies(add_two_ints_client beginner_tutorials_gencpp)