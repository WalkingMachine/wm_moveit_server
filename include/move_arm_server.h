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
#include <agile_grasp/grasp_hypothesis.h>
#include <agile_grasp/Grasp.h>
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


struct GraspEigen
{
    /**
    * \brief Default constructor.
    */
    GraspEigen() { }

    /**
    * \brief Constructor. Convert an agile_grasp::Grasp message to a GraspEigen struct.
    * \param the agile_grasp message
    */

    GraspEigen(const agile_grasp::Grasp& grasp)
    {
        tf::vectorMsgToEigen(grasp.axis, axis_);
        tf::vectorMsgToEigen(grasp.approach, approach_);
        tf::vectorMsgToEigen(grasp.center, center_);
        tf::vectorMsgToEigen(grasp.surface_center, surface_center_);

        approach_ = -1.0 * approach_; // make approach vector point away from handle centroid
        binormal_ = axis_.cross(approach_); // binormal (used as rotation axis to generate additional approach vectors)
    }

    Eigen::Vector3d center_; ///< the grasp position
    Eigen::Vector3d surface_center_; ///< the grasp position projected back onto the surface of the object
    Eigen::Vector3d axis_; ///< the hand axis
    Eigen::Vector3d approach_; ///< the grasp approach direction
    Eigen::Vector3d binormal_; ///< the vector orthogonal to the hand axis and the grasp approach direction
};

tf::Quaternion calculateHandOrientations(const GraspEigen& grasp);
int main(int argc, char **argv);
bool move( wm_moveit_server::moveRequest &req, wm_moveit_server::moveResponse &resp );
bool grasp( wm_moveit_server::pickRequest &req, wm_moveit_server::pickResponse &resp );
void waitForExecution( moveit::planning_interface::MoveGroupInterface group );




#endif //PROJECT_MOVE_ARM_ACTION_SERVER_H
