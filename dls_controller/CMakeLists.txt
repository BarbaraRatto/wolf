cmake_minimum_required(VERSION 2.8.3)
project(dls_controller)

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

# ROS
find_package(catkin REQUIRED tf controller_interface dls_hardware_interface realtime_tools sensor_msgs geometry_msgs message_generation)
# ADVR Packages
find_package(OpenSoT REQUIRED) #FIXME Do I really need it?
find_package(XCM REQUIRED) #FIXME Do I really need it?
find_package(cartesian_interface REQUIRED)


add_service_files(FILES
  DlsControllerServices.srv)

generate_messages()

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
    ${XCM_INCLUDE_DIRS}
    ${cartesian_interface_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/ForceOptimization.cpp src/controller.cpp src/IDProblem.cpp)
add_dependencies(${PROJECT_NAME} ${dls_controller_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenSoT_LIBRARIES} ${XCM_LIBRARIES} ${cartesian_interface_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roscpp rostest urdf controller_manager hardware_interface dls_hardware_interface hyqreal_description)
  include_directories(include ${catkin_INCLUDE_DIRS})

  add_executable(hyq_dummy test/hyq_dummy.cpp)
  target_link_libraries(hyq_dummy ${catkin_LIBRARIES})

  add_rostest_gtest(dls_controller_test
    test/dls_controller_test.test
    test/dls_controller_test.cpp)
  target_link_libraries(dls_controller_test ${catkin_LIBRARIES})
endif()
