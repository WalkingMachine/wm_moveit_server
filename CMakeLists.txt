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
    roscpp rospy
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
        FILES  move.srv pick.srv move_joints.srv
)

generate_messages(
        DEPENDENCIES geometry_msgs std_msgs agile_grasp
)

add_executable( moveit_server
        include/move_arm_server.h
        src/move_arm_server.cpp
        )


include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

target_link_libraries(moveit_server ${catkin_LIBRARIES})
