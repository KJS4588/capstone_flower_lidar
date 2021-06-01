cmake_minimum_required(VERSION 3.0.2)
project(lidar_practice)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
)

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

catkin_package()

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/lidar_practice.cpp
# )

add_executable(${PROJECT_NAME} src/node.cpp)

# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/graph.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

