cmake_minimum_required(VERSION 2.8)
PROJECT(iit-teleop-ros-pkg)

add_custom_target(  acm
                    COMMAND ${CMAKE_SOURCE_DIR}/teleop_urdf/script/create_acm.sh teleop)

add_custom_target(	remove_old_teleop_urdf_gazebo ALL
    				COMMAND ${CMAKE_COMMAND} -E remove_directory 
        			"~/.gazebo/models/teleop_urdf/")

add_custom_target(  remove_old_teleop_gazebo ALL
                    COMMAND ${CMAKE_COMMAND} -E remove_directory 
                    "~/.gazebo/models/teleop/")

add_custom_target(  compile_urdf_sdf ALL
                    COMMAND ${CMAKE_SOURCE_DIR}/teleop_urdf/script/create_urdf_srdf_sdf.sh teleop)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    add_custom_target(	copy_teleop_gazebo ALL
        				COMMAND ${CMAKE_COMMAND} -E copy_directory 
            			"${PROJECT_SOURCE_DIR}/teleop_gazebo/sdf/" 
            			"~/.gazebo/models/teleop/"
            			DEPENDS remove_old_teleop_gazebo)
endif()
