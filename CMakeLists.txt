cmake_minimum_required(VERSION 2.8.3)
project(wm_moveit_server)
SET(CMAKE_CXX_STANDARD 11)
add_definitions(-std=c++11)
find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_core
    moveit_ros_move_group
    moveit_ros_planning_interface
    std_msgs
    message_generation
    geometry_msgs
    std_srvs
    tf
    tf_conversions
    eigen_conversions
)

add_service_files(
        DIRECTORY srv
        FILES  move.srv move_joints.srv #pick.srv
)

generate_messages(
        DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs
    geometry_msgs
    roscpp
    moveit_core
    moveit_ros_move_group
    moveit_ros_planning_interface
    std_msgs
    geometry_msgs
    std_srvs
    tf
    tf_conversions
    eigen_conversions
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

add_executable( ${PROJECT_NAME}
        include/move_arm_server.h
        src/move_arm_server.cpp
        )

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
