cmake_minimum_required(VERSION 3.0.0)
project(wolf_controller)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wmaybe-uninitialized -Wuninitialized")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 nor C++14 support. Please use a different C++ compiler.")
endif()

# ROS
find_package(catkin REQUIRED tf controller_interface control_toolbox wolf_hardware_interface
    realtime_tools geometry_msgs sensor_msgs nav_msgs ddynamic_reconfigure message_generation rviz_visual_tools rt_logger)
# ADVR Packages
find_package(OpenSoT REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(ModelInterfaceRBDL REQUIRED)

add_message_files(FILES
                 CartesianTask.msg
                 PosturalTask.msg
                 ComTask.msg
                 ContactForces.msg
                 FootHolds.msg
                 TerrainEstimation.msg
                 FrictionCones.msg
                 CapturePoint.msg
)

add_service_files(
  FILES
  float32.srv
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenSoT_INCLUDE_DIRS}
    ${cartesian_interface_INCLUDE_DIRS}
    ${XBotInterface_INCLUDE_DIRS}
    ${ModelInterfaceRBDL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/controller.cpp src/id_problem.cpp src/footholds_planner.cpp src/state_estimator.cpp src/foot_trajectory_ellipse.cpp
    src/foot_trajectory_interface.cpp src/legs_kinematics.cpp src/impedance.cpp src/quadruped_robot.cpp src/com_planner.cpp src/terrain_estimator.cpp
    src/gait_generator.cpp src/foot_state_machine.cpp)
add_dependencies(${PROJECT_NAME} ${wolf_controller_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenSoT_LIBRARIES} ${cartesian_interface_LIBRARIES} ${XBotInterface_LIBRARIES} ${ModelInterfaceRBDL_LIBRARIES})

add_executable(plot_node src/plot_node.cpp)
add_dependencies(plot_node ${wolf_controller_EXPORTED_TARGETS})
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

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY resources/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources
  PATTERN ".svn" EXCLUDE)

catkin_install_python(PROGRAMS scripts/keyboard_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roscpp rostest urdf controller_manager hardware_interface wolf_hardware_interface std_msgs)
  find_package(OpenSoT REQUIRED)
  find_package(cartesian_interface REQUIRED)
  include_directories(include ${catkin_INCLUDE_DIRS})

  add_library(robot_dummy test/robot_dummy.cpp)
  target_link_libraries(robot_dummy ${catkin_LIBRARIES} ${PROJECT_NAME})

  add_executable(test_state_estimation test/test_state_estimation.cpp)
  target_link_libraries(test_state_estimation ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenSoT_LIBRARIES} ${cartesian_interface_LIBRARIES})

  #add_executable(test_id_problem test/test_id_problem.cpp)
  #target_link_libraries(test_id_problem ${catkin_LIBRARIES} ${PROJECT_NAME} gtest)

  #add_executable(test_controller test/test_controller.cpp )
  #target_link_libraries(test_controller ${catkin_LIBRARIES} ${PROJECT_NAME} gtest robot_dummy)

  add_rostest_gtest(test_controller
    test/test_controller.test
    test/test_controller.cpp)
  target_link_libraries(test_controller ${catkin_LIBRARIES} ${PROJECT_NAME} gtest robot_dummy)

  add_rostest_gtest(test_id_problem
    test/test_id_problem.test
    test/test_id_problem.cpp)
  target_link_libraries(test_id_problem ${catkin_LIBRARIES} ${PROJECT_NAME} gtest)
endif()
