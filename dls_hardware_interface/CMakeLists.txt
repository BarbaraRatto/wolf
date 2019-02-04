cmake_minimum_required(VERSION 2.8.3)
project(dls_hardware_interface)

find_package(catkin REQUIRED COMPONENTS roscpp hardware_interface)

find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    hardware_interface
  INCLUDE_DIRS
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${catkin_LIBRARIES}
)

#add_library(dls_robot_hw src/dls_robot_hw.cpp)
#target_link_libraries(dls_robot_hw hardware_interface)

#install(TARGETS dls_robot_hw
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
