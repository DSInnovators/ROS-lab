cmake_minimum_required(VERSION 3.0.2)
project(vehicle_data_visualization LANGUAGES CXX)

# # Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation)

# # # Declare ROS messages and services
add_message_files(DIRECTORY msg FILES Vehicle.msg)

# # Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# # Declare a catkin package
catkin_package()

# # Build talker and listener
include_directories(include 
  ${catkin_INCLUDE_DIRS}  
)

add_executable(plot src/plot.cc)
target_link_libraries(plot ${catkin_LIBRARIES})

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(plot ${PYTHON_LIBRARIES})