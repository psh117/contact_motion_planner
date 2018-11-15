cmake_minimum_required(VERSION 2.8.3)
project(contact_motion_planner)

add_compile_options(-std=c++11)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
message(STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(fcl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES contact_motion_planner
  CATKIN_DEPENDS roscpp tf2 tf2_geometry_msgs
  #  DEPENDS system_lib
)

#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(contact_motion_planner
  src/contact_planner.cpp
  src/planning_scene.cpp
  src/contact_model_graph.cpp

  src/contact/contact.cpp
  src/contact_model/contact_model.cpp
  src/contact_model/box_contact_model.cpp
  src/contact_model/visualization/contact_model_visualization.cpp

  src/robot_dynamics/robot_dynamics_model.cpp

  src/solver/contact_optimization_solver.cpp
  src/solver/contact_optimization.cpp

  src/solver/constraint/constraint_base.cpp
  src/solver/constraint/constraint_equality.cpp
  src/solver/constraint/constraint_inequality.cpp
)

add_dependencies(contact_motion_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#set_property(TARGET contact_motion_planner PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries (contact_motion_planner
  qpOASES
  fcl
)

add_executable(example src/example.cpp)
target_link_libraries (example
  ${catkin_LIBRARIES}
  contact_motion_planner
  fcl
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/utest.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
    ${catkin_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

