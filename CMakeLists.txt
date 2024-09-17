cmake_minimum_required(VERSION 2.8.3)
project(om_modbus_master)

add_compile_options(-std=c++11)
add_compile_options(-Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    tf
    tf2
    tf2_ros
    nav_msgs
)

# # Generate messages in the 'msg' folder
add_message_files(
    FILES
    om_query.msg
    om_response.msg
    om_state.msg
)

# # Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include

    # LIBRARIES om_modbus_master

    # add
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

# # Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(om_modbusRTU_node src/om_node.cpp src/om_first_gen.cpp src/om_ros_message.cpp src/om_base.cpp src/om_second_gen.cpp src/om_broadcast.cpp src/om_idshare_mode.cpp)

add_executable(om_ros_node sample/cpp/MobiCon/om_changer.cpp)

# add
add_dependencies(om_modbusRTU_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(om_modbusRTU_node ${catkin_LIBRARIES})

add_dependencies(om_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(om_ros_node ${catkin_LIBRARIES})

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/om_changer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)