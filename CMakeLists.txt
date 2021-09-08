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
#  realtime_tools
#  dynamic_reconfigure
)

# Dynamics reconfigure
#generate_dynamic_reconfigure_options(
#  cfg/PassivityControllerParams.cfg
#  )

# message
#add_message_files(FILES ControllerJointState.msg)
#generate_messages(DEPENDENCIES std_msgs)

# include
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    angles
    controller_interface
    control_msgs
    urdf
    kdl_parser
 # LIBRARIES ${PROJECT_NAME}
)

#add_library(${PROJECT_NAME}
#  src/time_delay_controller.cpp
#  src/gravity_comp_controller.cpp
#  src/gravity_comp_alternative_controller.cpp
#  src/computed_torque_controller.cpp
#  src/computed_torque_clik_controller.cpp
#  src/adaptive_variable_impedance_controller.cpp
#  src/passivity_controller.cpp
#)
#


add_executable(forward_kinematics src/forward_kinematics.cpp)
target_link_libraries(forward_kinematics ${catkin_LIBRARIES})
add_dependencies(forward_kinematics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(forward_kinematics ${PROJECT_NAME}_gencfg)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS forward_kinematics
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

