cmake_minimum_required(VERSION 2.8.3)
project(controller)

set(OsqpEigen_DIR /usr/local/lib/cmake/OsqpEigen/)

# # Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  tf_conversions
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  OsqpEigen
  osqp
  message_generation
  quadrotor_msgs
)

add_message_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -Wall")
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs

  # LIBRARIES cpr_closed_loop_controller
  # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  /usr/local/include/osqp
  ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

link_directories(
  ${catkin_LIB_DIRS}
  /usr/local/lib
)

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

include_directories(OsqpEigen::OsqpEigen)
include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(mpc_controller_node
  ${SRC_DIR}/mpc_controller_node.cpp
  ${SRC_DIR}/MPCController.cpp
  ${INCLUDE_DIR}/MPCController.h)

# # Add cmake target dependencies of the executable
# # same as for the library above
add_dependencies(mpc_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Specify libraries to link a library or executable target against
target_link_libraries(mpc_controller_node
  ${catkin_LIBRARIES}
  OsqpEigen::OsqpEigen
)
