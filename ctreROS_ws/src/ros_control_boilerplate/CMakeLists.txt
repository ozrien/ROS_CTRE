cmake_minimum_required(VERSION 2.8.3)
project(ros_control_boilerplate)

include ("../cmake_modules/CMakeOpt.cmake")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  realtime_tools
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  actionlib
  urdf
  std_msgs
  sensor_msgs
  rosparam_shortcuts
  talon_interface
  message_generation
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
find_package(Gflags REQUIRED)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
	hardware_interface
	controller_manager
	control_msgs
	trajectory_msgs
	actionlib
	urdf
	std_msgs
	sensor_msgs
	rosparam_shortcuts
	talon_interface
	message_runtime
  LIBRARIES
)

###########
## Build ##
###########

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

include_directories(
  include/
  $ENV{HOME}/CTRE/cpp/include
  ${catkin_INCLUDE_DIRS}
  ${Gflags_INCLUDE_DIRS}
)

# Sim version of main control
# executable.  For now this just prints debugging
# info for testing code which doens't actually hit
# RoboRIO or CAN hardware. Eventually expand into
# controlling some sort of sim?
add_executable(ctrerobot_sim_main 
	src/ctrerobot_sim_main.cpp
	src/ctrerobot_sim_interface.cpp
	src/ctre_robot_interface.cpp
	src/generic_hw_control_loop.cpp
)

target_link_libraries(ctrerobot_sim_main
	${catkin_LIBRARIES}
)

# Forces messages to be generated before
# the target which uses them
add_dependencies(ctrerobot_sim_main
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

# Install executable on the off chance you'd ever
# want to run sim on the RoboRIO
install(TARGETS
  ctrerobot_sim_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#if 
# CTRERobot Hardware Interface only

# Main control executable
add_executable(ctrerobot_hw_main 
	src/ctrerobot_hw_main.cpp
	src/ctrerobot_hw_interface.cpp
	src/ctre_robot_interface.cpp
	src/generic_hw_control_loop.cpp
)


if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    set(LIBCTRE_PHOENIX_PATH $ENV{HOME}/CTRE/cpp/lib/arm-linux-gnueabihf/libCTRE_Phoenix.a)
    set(LIBCTRE_PHOENIXCCI_PATH $ENV{HOME}/CTRE/cpp/lib/arm-linux-gnueabihf/libCTRE_PhoenixCCI.a)

elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX1/TX2 (not set up right now)
    set(LIBCTRE_PHOENIX_PATH $ENV{HOME}/CTRE/cpp/lib/aarch64-linux-gnu/libCTRE_Phoenix.a)
    set(LIBCTRE_PHOENIXCCI_PATH $ENV{HOME}/CTRE/cpp/lib/aarch64-linux-gnu/libCTRE_PhoenixCCI.a)
else () #assumed x86
    set(LIBCTRE_PHOENIX_PATH $ENV{HOME}/CTRE/cpp/lib/x86/libCTRE_Phoenix.a)
    set(LIBCTRE_PHOENIXCCI_PATH $ENV{HOME}/CTRE/cpp/lib/x86/libCTRE_PhoenixCCI.a)
endif()

target_link_libraries(ctrerobot_hw_main
	${catkin_LIBRARIES}
    ${LIBCTRE_PHOENIX_PATH}
    ${LIBCTRE_PHOENIXCCI_PATH}
)

## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## talon controller messages to be built before the
## talon controller code tries to use them
add_dependencies(ctrerobot_hw_main
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
## Install ------------------------------------------------------------

# Install executables
#endif()

# Test trajectory generator node
add_executable(${PROJECT_NAME}_test_trajectory src/tools/test_trajectory.cpp)
set_target_properties(${PROJECT_NAME}_test_trajectory PROPERTIES OUTPUT_NAME test_trajectory PREFIX "")
target_link_libraries(${PROJECT_NAME}_test_trajectory
  ${catkin_LIBRARIES}
)

## TOOLS ------------------------------------------------------

# Tool for analyzing controller performance
add_executable(${PROJECT_NAME}_controller_to_csv_main 
	src/tools/controller_to_csv_main.cpp
	src/tools/controller_to_csv.cpp)
set_target_properties(${PROJECT_NAME}_controller_to_csv_main PROPERTIES OUTPUT_NAME controller_to_csv_main PREFIX "")
target_link_libraries(${PROJECT_NAME}_controller_to_csv_main
  ${Gflags_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Tool for analyzing controller performance
add_library(csv_to_controller src/tools/csv_to_controller.cpp)
target_link_libraries(csv_to_controller
  ${catkin_LIBRARIES}
)
add_executable(${PROJECT_NAME}_csv_to_controller_main 
	src/tools/csv_to_controller_main.cpp
	src/tools/csv_to_controller.cpp)
set_target_properties(${PROJECT_NAME}_csv_to_controller_main PROPERTIES OUTPUT_NAME csv_to_controller_main PREFIX "")
target_link_libraries(${PROJECT_NAME}_csv_to_controller_main
  ${Gflags_LIBRARIES}
  ${catkin_LIBRARIES}
)


## Install ------------------------------------------------------------
install(TARGETS
  ctrerobot_hw_main
  ctrerobot_sim_main
    ${PROJECT_NAME}_test_trajectory
    ${PROJECT_NAME}_controller_to_csv_main
    ${PROJECT_NAME}_csv_to_controller_main
    
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY 
	include/${PROJECT_NAME}
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY
	launch
	config
	urdf
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
