cmake_minimum_required(VERSION 3.0.2)
project(vehicle_data_visualization LANGUAGES CXX)

# # Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy auto_vehicle_msgs)
find_package(PythonLibs REQUIRED)

# # # # Declare ROS messages and services
# add_message_files(DIRECTORY msg FILES Status.msg)

# # Generate added messages and services
# generate_messages(DEPENDENCIES )

# # Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp auto_vehicle_msgs
)

# # Build talker and listener
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

set(NODE_NAME "plot")

add_executable(${NODE_NAME} src/plot.cc)
target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})

add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
