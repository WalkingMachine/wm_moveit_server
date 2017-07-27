////
//// Created by philippe on 02/07/17.
////
//
#ifndef PROJECT_MOVE_ARM_ACTION_SERVER_H
#define PROJECT_MOVE_ARM_ACTION_SERVER_H
//
//
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_planners_ompl/OMPLDynamicReconfigureConfig.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <wm_moveit_server/move.h>
#include <wm_moveit_server/pick.h>
#include <wm_moveit_server/move_joints.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

//
//moveit::planning_interface::MoveGroupInterface group;
//
//
//
//
////class move_arm_action_server {
////
////};
//
//

int main(int argc, char **argv);
bool move( wm_moveit_server::moveRequest &req, wm_moveit_server::moveResponse &resp );
bool move_joint( wm_moveit_server::move_jointsRequest &req, wm_moveit_server::move_jointsResponse &resp );
void waitForExecution( moveit::planning_interface::MoveGroupInterface group );




#endif //PROJECT_MOVE_ARM_ACTION_SERVER_H
