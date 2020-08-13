#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "std_msgs/String.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Image.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include <iostream>
#include <fstream>
#include <string>


class Robot
{
  private:
    const std::string PLANNING_GROUP = "manipulator";
    const moveit::core::JointModelGroup* joint_model_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group = moveit::planning_interface::MoveGroupInterface(this->PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::Pose home_pose;
    geometry_msgs::Pose workspace_pose;
    sensor_msgs::Image current_img;
    ros::NodeHandle robot_node;
    ros::ServiceClient gripper_on_client = robot_node.serviceClient<std_srvs::Empty>("/ur5/vacuum_gripper/on");
    ros::ServiceClient gripper_off_client = robot_node.serviceClient<std_srvs::Empty>("/ur5/vacuum_gripper/off");
    ros::ServiceClient spawn_model_client = robot_node.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    ros::ServiceClient delete_model_client = robot_node.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    ros::ServiceClient get_model_state_client = robot_node.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::ServiceClient set_model_state_client = robot_node.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceServer order_service = robot_node.advertiseService("/pack_order", &Robot::fulfillOrderCallback, this);
    ros::Subscriber image_subscriber = robot_node.subscribe("/rrbot/camera1/image_raw", 1, &Robot::imageCallback, this);
    ros::Subscriber image_open_cv_subscriber = robot_node.subscribe("/image_converter/output_video", 1, &Robot::imageCallbackOpenCV, this);
    bool isStopImage = false;

    bool fulfillOrderCallback(std_srvs::SetBoolRequest  &req, std_srvs::SetBoolResponse &res);
    // void imageCallback(const sensor_msgs::Image& msg);
    void imageCallback(const sensor_msgs::ImagePtr& msg);
    void imageCallbackOpenCV(const sensor_msgs::ImagePtr& msg);

  
  public:
    Robot();
    bool home();
    bool goToWorkspace();
    bool move(geometry_msgs::Pose target_pose);
    bool moveConstrained(geometry_msgs::Pose target_pose);
    bool gripperOn();
    bool gripperOff();
    bool pickUpAndDrop(geometry_msgs::Pose target_pose, geometry_msgs::Pose drop_pose);
    bool spawnObject(std::string object_name);
    bool deleteObject(std::string object_name);
    bool setModelPose(std::string model_name, geometry_msgs::Pose target_pose);
    geometry_msgs::Pose getModelPose(std::string model_name);
    
    
    
};
