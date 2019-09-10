cmake_minimum_required(VERSION 2.8.3)
project(wb_controller)

# C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# ROS
find_package(catkin REQUIRED tf controller_interface control_toolbox dls_hardware_interface
    realtime_tools geometry_msgs sensor_msgs nav_msgs dynamic_reconfigure message_generation rviz_visual_tools)
# ADVR Packages
find_package(OpenSoT REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(PkgConfig)
find_package(RBDL REQUIRED)
pkg_check_modules(RBDL REQUIRED rbdl=2.5.0)

generate_dynamic_reconfigure_options(
  cfg/controller.cfg
  cfg/problem.cfg
  cfg/taskCartesian.cfg
  cfg/taskGeneric.cfg
  cfg/taskPostural.cfg
)

add_message_files(FILES
                 CartesianTask.msg
                 JointsTask.msg
                 ContactForces.msg
                 LoggerNumeric.msg)

generate_messages(
    DEPENDENCIES
    geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenSoT_INCLUDE_DIRS}
    ${cartesian_interface_INCLUDE_DIRS}
    ${RBDL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/controller.cpp src/id_problem.cpp src/commands_interface.cpp src/state_estimator.cpp)
add_dependencies(${PROJECT_NAME} ${wb_controller_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${RBDL_LIBRARIES} ${catkin_LIBRARIES} ${OpenSoT_LIBRARIES} ${cartesian_interface_LIBRARIES})

add_executable(plot_node src/plot_node.cpp)
add_dependencies(plot_node ${wb_controller_EXPORTED_TARGETS})
target_link_libraries(plot_node ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} plot_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES controller_plugin.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})  

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY plot_juggler_files/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plot_juggler_files
  PATTERN ".svn" EXCLUDE)


#if (CATKIN_ENABLE_TESTING)
#  find_package(catkin REQUIRED COMPONENTS roscpp rostest urdf controller_manager hardware_interface dls_hardware_interface std_msgs)
#  find_package(OpenSoT REQUIRED)
#  find_package(cartesian_interface REQUIRED)
#  include_directories(include ${catkin_INCLUDE_DIRS})

#  add_executable(robot_dummy test/robot_dummy.cpp)
#  target_link_libraries(robot_dummy ${catkin_LIBRARIES})

#  add_executable(test_state_estimation test/test_state_estimation.cpp)
#  target_link_libraries(test_state_estimation ${catkin_LIBRARIES} ${OpenSoT_LIBRARIES} ${cartesian_interface_LIBRARIES})

#  add_rostest_gtest(wb_controller_test
#    test/wb_controller_test.test
#    test/wb_controller_test.cpp)
#  target_link_libraries(wb_controller_test ${catkin_LIBRARIES})
#endif()
