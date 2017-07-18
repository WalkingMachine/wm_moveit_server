////
//// Created by philippe on 02/07/17.
////

#include <wm_moveit_server/move_arm_server.h>
#include <agile_grasp/Grasps.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <grasp_selection/SelectGrasps.h>

// taken from the grasp selection package





bool grasp( wm_moveit_server::pickRequest &req, wm_moveit_server::pickResponse &resp ) {
    moveit::planning_interface::MoveGroupInterface group(req.move_group);
    bool success = false;
    group.allowReplanning(false);
    //group.setNumPlanningAttempts(5);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // call the grasp_selection service
    grasp_selection::SelectGrasps selector;
    selector.request.hand_pose = group.getCurrentPose().pose;
    ros::service::waitForService( "grasp_selection" );
    ros::service::call( "grasp_selection", selector );
    selector.response.grasps.grasps[0].pose;



    int length = (int) selector.response.grasps.grasps.size();
    for (int i = 0; i < length; i++) {

        group.setPoseTarget( selector.response.grasps.grasps[i].pose );

        if (group.plan(plan).val == 1) {
            success = true;
            break;
        }
    }
    if (success) {
        group.asyncExecute(plan);
        waitForExecution(group);
    }
    resp.success = (char)(success ? 1 : 0);

    return true;
}





bool move( wm_moveit_server::moveRequest &req, wm_moveit_server::moveResponse &resp )
{
    try {
        moveit::planning_interface::MoveGroupInterface group(req.move_group);

        ROS_INFO("Stoping previous move execution");
        group.stop();
        group.clearPoseTargets();
        group.setPoseTarget(req.pose);
        group.setGoalTolerance( 0.01 );
        group.setGoalJointTolerance(0.01);
        group.setGoalPositionTolerance(0.01);
        ROS_INFO("Starting move execution");
        group.allowReplanning( true );
        group.setNumPlanningAttempts( 5 );
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        int attempt = 1;
        do{
            attempt ++;
            if ( attempt > 2 ){
                ROS_ERROR( "target couldn't be planned" );
                resp.success = 0;
                return true;
            }
        }while(group.plan( plan ).val != 1 );
        group.execute( plan );
        resp.success = 1;
        waitForExecution( group );

        ROS_INFO("Move result: %d",resp.success);
    } catch ( __exception ex ){
        resp.success = 0;
    }
    return true;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "move_arm_server");

    ros::AsyncSpinner sp( 4 );
    sp.start();

    ros::NodeHandle nh;

    ros::ServiceServer serviceMove = nh.advertiseService( "move_arm", move );
    ros::ServiceServer servicePick = nh.advertiseService( "pick", grasp );
    ROS_INFO("Ready to move.");
    //ros::spin();
    while ( ros::ok()){}

    return 0;
}








tf::Quaternion calculateHandOrientations(const GraspEigen& grasp)
{
    // calculate first hand orientation
    Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
    R.col(0) = -1.0 * grasp.approach_;
    R.col(1) = grasp.axis_;
    R.col(2) << R.col(0).cross(R.col(1));

    // rotate by 180deg around the grasp approach vector to get the "opposite" hand orientation
    Eigen::Transform<double, 3, Eigen::Affine> T(Eigen::AngleAxis<double>(3.14159, grasp.approach_));

    // convert Eigen rotation matrices to TF quaternions and normalize them
    tf::Matrix3x3 TF1;
    tf::matrixEigenToTF(R, TF1);

    tf::Quaternion quat1;
//    TF1.getRotation(quat1);
    quat1.normalize();

    return quat1;
}

void waitForExecution( moveit::planning_interface::MoveGroupInterface group ){
    double dist = 0;
    auto Tpos = group.getPoseTarget().pose;
    do {
        auto Cpos = group.getCurrentPose().pose;
        double dx2 = (Cpos.position.x-Tpos.position.x)*(Cpos.position.x-Tpos.position.x);
        double dy2 = (Cpos.position.y-Tpos.position.y)*(Cpos.position.y-Tpos.position.y);
        double dz2 = (Cpos.position.z-Tpos.position.z)*(Cpos.position.z-Tpos.position.z);
        dist = sqrt( dx2+dy2+dz2 );
        double Cw = Cpos.orientation.w;
        double Tw = Tpos.orientation.w;
        if ( Tw != 0 && Cw != 0 ) {
            dx2 = (Cpos.orientation.x / Cw - Tpos.orientation.x / Tw) *
                  (Cpos.orientation.x / Cw - Tpos.orientation.x / Tw);
            dy2 = (Cpos.orientation.y / Cw - Tpos.orientation.y / Tw) *
                  (Cpos.orientation.y / Cw - Tpos.orientation.y / Tw);
            dz2 = (Cpos.orientation.z / Cw - Tpos.orientation.z / Tw) *
                  (Cpos.orientation.z / Cw - Tpos.orientation.z / Tw);
        }
        dist += sqrt( dx2+dy2+dz2 );
        usleep(100000);
        ROS_ERROR( "dist: %f", dist );
    }while ( dist > 0.05 );
}