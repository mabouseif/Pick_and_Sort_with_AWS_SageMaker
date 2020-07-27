#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/GetModelState.h"

#include <ros/ros.h>

// #include <moveit_visual_tools/moveit_visual_tools.h>


// check this out https://github.com/lihuang3/ur5_ROS-Gazebo
// http://wiki.ros.org/action/show/universal_robots?action=show&redirect=universal_robot
// http://wiki.ros.org/ur_gazebo



class Robot
{
  private:
    const std::string PLANNING_GROUP = "manipulator";
    const moveit::core::JointModelGroup* joint_model_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group = moveit::planning_interface::MoveGroupInterface(this->PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::Pose home_pose;

  
  public:
    Robot();
    bool move(geometry_msgs::Pose target_pose);
    bool home();
};

Robot::Robot()
{
    // this->move_group(this->PLANNING_GROUP);
    this->joint_model_group = this->move_group.getCurrentState()->getJointModelGroup(this->PLANNING_GROUP);

    ROS_INFO_NAMED("tutorial", "End effector link: %s", this->move_group.getEndEffectorLink().c_str());

    this->home_pose.position.x = 0.000118748;
    this->home_pose.position.y = 0.19145;
    this->home_pose.position.z = 0.6; //1.00106;
    this->home_pose.orientation.x = 0.000101546;
    this->home_pose.orientation.y = 0.000101546;
    this->home_pose.orientation.z = 0.707142; 
    this->home_pose.orientation.w = 0.707072;

    this->home();
    ros::Duration(1.0).sleep();
}

bool Robot::move(geometry_msgs::Pose target_pose)
{
    this->move_group.setPoseTarget(target_pose);

    // Note that we are just planning, not asking move_group to actually move the robot.
    bool success = (this->move_group.plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Message", "Planning result: %d", success);

    if (!success)
        ROS_INFO_NAMED("Error", "Could not go to target pose");
    else
    {
        this->move_group.move();
        ROS_INFO_NAMED("Message", "Target pose reached!");
    }

    this->move_group.clearPoseTargets();
    this->move_group.setStartStateToCurrentState();

    return success;
        
}

bool Robot::home()
{
    this->move(this->home_pose);
}



geometry_msgs::Pose target_pose1;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(0);
    spinner.start();


    ros::ServiceClient client = node_handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    client.waitForExistence();

    gazebo_msgs::GetModelState srv;
    srv.request.model_name = "red_box"; //atoll("red_box");


    Robot robot;

    if (client.call(srv))
    {
        ROS_INFO("Service Call Result: %d", srv.response.success);
        ROS_INFO("Service Call Status: %s", srv.response.status_message);
        std::cout << srv.response.status_message << std::endl;
        target_pose1 = srv.response.pose;
        
        robot.move(target_pose1);
    }
    else
    {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }


    ros::waitForShutdown();


    return 0;

}

