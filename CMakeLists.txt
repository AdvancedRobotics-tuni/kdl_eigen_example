cmake_minimum_required(VERSION 2.8.3)
project(kdl_eigen_example)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
find_package(catkin REQUIRED COMPONENTS
  angles
  controller_interface
  control_msgs
  urdf
  kdl_parser
  backward_ros
)

# include
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    angles
    controller_interface
    control_msgs
    urdf
    kdl_parser
)



add_executable(forward_kinematics src/forward_kinematics.cpp)
target_link_libraries(forward_kinematics ${catkin_LIBRARIES})
add_dependencies(forward_kinematics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS forward_kinematics
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

