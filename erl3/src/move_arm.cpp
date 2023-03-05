/** @package erl3
*
* \file move_arm.cpp
* \brief this node implements the configuration poses of the robot.
*
* \author Roberta Reho
* \version 1.0
* \date 04/03/2023
*
* \details
*
* Subscribes to: <BR>
*     None
*
* Publishes to: <BR>
*     None
*
* Serivces: <BR>
*     arm_pose
*
* Client Services: <BR>
*     None
*
* Action Client: <BR>
*     None
*
* Description: <BR>
* This node implements the callback function of the arm_pose service that moves the robot arm 
* to a specified position.
*/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <erl3/Pose.h>
// positions: default - high_reach - low_reach

/**
* \brief Callback function of the arm_pose service.
* \param req: pose request
* \param res: pose response
* \return: true
*
* This function the callback function of the arm_pose service that moves the robot arm 
* to a specified position. It uses the MoveIt library and sets various parameters 
* for planning and execution, including the planner ID, planning time, and goal tolerances.
* The function returns true if the arm is successfully moved to the specified position 
* false otherwise.
*
*/
bool moveArm(erl3::Pose::Request &req, erl3::Pose::Response &res){
    
    if (req.position != "default" && req.position != "high_reach" && req.position != "low_reach"){
        std::cout<<"WARNING: asked for wrong position!"<<std::endl;
        return false;
    }
    moveit::planning_interface::MoveGroupInterface group("arm");
    group.setEndEffectorLink("cluedo_link");
    group.setPoseReferenceFrame("base_link");
    group.setPlannerId("RRTstar");
    group.setNumPlanningAttempts(10);
    group.setPlanningTime(10.0);
    group.allowReplanning(true);
    group.setGoalJointTolerance(0.0001);
    group.setGoalPositionTolerance(0.0001);
    group.setGoalOrientationTolerance(0.001);
    group.setNamedTarget(req.position);
    group.move();
    
    return true;
}



int main(int argc, char **argv){
    ros::init(argc, argv, "move_arm");
    ros::NodeHandle nh;
    
    ros::ServiceServer pose_srv = nh.advertiseService("arm_pose", moveArm);
    
    ros::AsyncSpinner spinner(100);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
