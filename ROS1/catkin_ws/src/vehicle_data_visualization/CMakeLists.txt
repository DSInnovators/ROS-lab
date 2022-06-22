cmake_minimum_required(VERSION 3.0.2)
project(vehicle_data_visualization LANGUAGES CXX)

# # Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation)
find_package(PythonLibs REQUIRED)

# # # Declare ROS messages and services
add_message_files(DIRECTORY msg FILES Vehicle.msg)

# # Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# # Declare a catkin package
catkin_package()

# # Build talker and listener
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

set(NODE_NAME "plot")

add_executable(${NODE_NAME} src/plot.cc)
target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})

add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
