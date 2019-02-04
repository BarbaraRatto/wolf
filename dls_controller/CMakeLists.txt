cmake_minimum_required(VERSION 2.8.3)
project(dls_controller)

find_package(catkin REQUIRED controller_interface dls_hardware_interface)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest urdf controller_manager hardware_interface dls_hardware_interface hyqreal_description)
  include_directories(include ${catkin_INCLUDE_DIRS})

  add_executable(hyq_dummy test/hyq_dummy.cpp)
  target_link_libraries(hyq_dummy ${catkin_LIBRARIES})

  add_rostest_gtest(dls_controller_test
    test/dls_controller_test.test
    test/dls_controller_test.cpp)
  target_link_libraries(dls_controller_test ${catkin_LIBRARIES})
endif()
