cmake_minimum_required(VERSION 3.0.2)
project(project)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  kortex_driver
  message_generation
  roscpp
  rospy
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/../kortex_api/include/client
  ${PROJECT_SOURCE_DIR}/../kortex_api/include/client_stubs
  ${PROJECT_SOURCE_DIR}/../kortex_api/include/messages
  ${PROJECT_SOURCE_DIR}/../kortex_api/include/common
)

add_executable(arm_node src/arm_node.cpp src/ArmControl.cpp)
add_dependencies(arm_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_node ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(gen_path_node src/gen_path_node.cpp)
add_dependencies(gen_path_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gen_path_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS script/ArmControl.py
                               script/gen_path.py
                               script/gen_circle.py
                               script/test.py
                               script/circle.py
                               script/lqr.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})