cmake_minimum_required(VERSION 2.8.3)
project(gen3kinova_description)

find_package(catkin REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rrbot_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY urdfs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdfs
  PATTERN ".svn" EXCLUDE)
