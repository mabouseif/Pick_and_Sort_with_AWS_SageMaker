#include "robot.h"

Robot::Robot()
{
    // Wait for services to be up
    gripper_on_client.waitForExistence();
    gripper_off_client.waitForExistence();
    spawn_model_client.waitForExistence();
    delete_model_client.waitForExistence();
    get_model_state_client.waitForExistence();
    set_model_state_client.waitForExistence();

    // Prepare MoveIt objects
    this->joint_model_group = this->move_group.getCurrentState()->getJointModelGroup(this->PLANNING_GROUP);

    ROS_INFO_NAMED("INFO", "Reference frame: %s", this->move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("INFO", "End effector link: %s", this->move_group.getEndEffectorLink().c_str());
    
    // Set home pose
    this->home_pose.position.x = 0.000118748;
    this->home_pose.position.y = 0.19145;
    this->home_pose.position.z = 1.00106; // 0.6; //1.00106;
    this->home_pose.orientation.x = 0.000101546;
    this->home_pose.orientation.y = 0.000101546;
    this->home_pose.orientation.z = 0.707142; 
    this->home_pose.orientation.w = 0.707072;

    this->workspace_pose.position.x = -0.500656408077;
    this->workspace_pose.position.y = 0.0501001578675;
    this->workspace_pose.position.z = 0.242487565229;
    this->workspace_pose.orientation.x = 0.5;
    this->workspace_pose.orientation.y = 0.5;
    this->workspace_pose.orientation.z = -0.5;
    this->workspace_pose.orientation.w = 0.5;

    // Home robot
    this->home();
    
}

// Move to pose
bool Robot::move(geometry_msgs::Pose target_pose)
{
    // Set pose for move group
    this->move_group.setPoseTarget(target_pose);

    // Plan to pose
    // Note that we are just planning, not asking move_group to actually move the robot.
    bool success = (this->move_group.plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Message", "Planning result: %d", success);

    if (success)
    {
        // Move to pose
        this->move_group.move();
        ROS_INFO_NAMED("Message", "Target pose reached!");
    }
    else
        ROS_INFO_NAMED("Error", "Could not go to target pose");

    // Clear
    this->move_group.clearPoseTargets();
    this->move_group.setStartStateToCurrentState();

    // Delay 
    ros::Duration(0.5).sleep();

    return success;
}


// Home robot
bool Robot::home()
{
    bool success = this->move(this->home_pose);
    return success;
}


// Go to workspace pose
bool Robot::goToWorkspace()
{
    bool success = this->move(this->workspace_pose);
    return success;
}


// Move to pose with constraints (Needs fixing)
bool Robot::moveConstrained(geometry_msgs::Pose target_pose)
{

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "ee_link";
    ocm.header.frame_id = "base_link";
    ocm.orientation.x = 0.5;
    ocm.orientation.y = 0.5;
    ocm.orientation.z = -0.5;
    ocm.orientation.w = 0.5;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    this->move_group.setPathConstraints(test_constraints);

    // We will reuse the old goal that we had and plan to it. Note that this 
    // will only work if the current state already satisfies the path constraints. 
    // So, we need to set the start state to a new pose.

    ros::Duration(2).sleep();
    robot_state::RobotState start_state(*this->move_group.getCurrentState());
    geometry_msgs::Pose start_pose2 = this->move_group.getCurrentPose().pose;;
    start_pose2.position.z += 0.1;
    bool suc = start_state.setFromIK(joint_model_group, start_pose2);
    move_group.setStartState(start_state);
    printf("Result is: %d\n", suc);


    // Now we will plan to the earlier pose target from the new start
    // state that we have just created.
    move_group.setPoseTarget(target_pose);

    // Planning with constraints can be slow because every sample must 
    // call an inverse kinematics solver. Lets increase the planning time 
    // from the default 5 seconds to be sure the planner has enough time to succeed.
    move_group.setPlanningTime(60.0);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

    move_group.execute(my_plan);

    std::cout << "Num objects: " << this->planning_scene_interface.getObjects().size() << std::endl;
    for (auto x: this->planning_scene_interface.getObjects())
        std::cout << x.first << std::endl;
    
    return success;

}


// Open gripper
bool Robot::gripperOn()
{
    std_srvs::Empty srv;
    bool success = this->gripper_on_client.call(srv);
    ros::Duration(1.0);

    return success;
}


// Close gripper
bool Robot::gripperOff()
{
    std_srvs::Empty srv;
    bool success = this->gripper_off_client.call(srv);
    ros::Duration(1.0);

    return success;
}


// Pick up object
bool Robot::pickUp(geometry_msgs::Pose target_pose, geometry_msgs::Pose drop_pose)
{
    target_pose.position.z += 0.1;
    this->move(target_pose);

    target_pose.position.z -= 0.1;
    this->move(target_pose);

    this->gripperOn();
    
    target_pose.position.z += 0.1;
    this->move(target_pose);

}


// Discard object
bool Robot::discardOrder(geometry_msgs::Pose target_pose, geometry_msgs::Pose drop_pose)
{

    drop_pose.position.y += 0.1;

    drop_pose.position.y += 0.2;
    this->move(drop_pose);

    // drop_pose.position.z += 0.1;
    // this->move(drop_pose);

    drop_pose.position.z -= 0.2;
    this->move(drop_pose);

    this->gripperOff();

    drop_pose.position.z += 0.1;

    this->deleteObject("red_box");
    this->deleteObject("blue_box");
    srand (time(NULL));
    this->spawnObject((rand() % 2) ? "red_box" : "blue_box");

    this->move(drop_pose);

    bool success = this->goToWorkspace();
}



// Dispatch object
bool Robot::dispatchOrder(geometry_msgs::Pose target_pose, geometry_msgs::Pose drop_pose)
{


    moveit_msgs::OrientationConstraint ocm;
    moveit_msgs::Constraints constraints;
    ocm.link_name = "ee_link";
    ocm.header.frame_id = "base_link";
    ocm.weight = 1.0;
    ocm.orientation = this->move_group.getCurrentPose().pose.orientation;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 2.0 * 3.14;

    
    constraints.orientation_constraints.push_back(ocm);
    this->move_group.setPathConstraints(constraints);

    drop_pose.position.y += 0.4; // 0.35;
    drop_pose.position.x += 0.5;
    this->move(drop_pose);

    this->move_group.clearPathConstraints();

    // drop_pose.position.z += 0.1;
    // this->move(drop_pose);

    drop_pose.position.z -= 0.1;
    this->move(drop_pose);

    this->gripperOff();

    drop_pose.position.z += 0.1;

    this->deleteObject("red_box");
    this->deleteObject("blue_box");
    srand (time(NULL));
    this->spawnObject((rand() % 2) ? "red_box" : "blue_box");

    this->move(drop_pose);

    // bool success = this->goToWorkspace();
    bool success = this->home();

}





// Spawn random object in workspace
bool Robot::spawnObject(std::string object_name)
{
    std::string path;
    if (object_name == "blue_box")
    {
        path = "/home/mohamed/catkin_ws/src/pick_and_sort/urdf/blue_box.urdf";
    }
    else
    {
        path = "/home/mohamed/catkin_ws/src/pick_and_sort/urdf/red_box.urdf";
    }

    std::ifstream ifs(path);
    std::string content( (std::istreambuf_iterator<char>(ifs) ),
                        (std::istreambuf_iterator<char>()    ) );
    
    geometry_msgs::Pose pose;
    pose.position.x = -0.5; 
    pose.position.y = 0.05;
    pose.position.z = 0.6;


    gazebo_msgs::SpawnModel spawn_model_srv;
    spawn_model_srv.request.model_name = object_name;
    spawn_model_srv.request.initial_pose = pose;
    spawn_model_srv.request.model_xml = content;
    // spawn_model_srv.request.robot_namespace = 
    
    
    if (this->spawn_model_client.call(spawn_model_srv))
        ROS_INFO_NAMED("INFO", "Successfully spawned %s", spawn_model_srv.request.model_name.c_str());
    else
        ROS_INFO_NAMED("ERROR", "Could not spawn %s", spawn_model_srv.request.model_name.c_str());

    ros::Duration(0.5).sleep();

    return true;

}


// Delete object from scene
bool Robot::deleteObject(std::string object_name)
{
    gazebo_msgs::DeleteModel delete_model_srv;
    delete_model_srv.request.model_name = object_name;
    
    if (this->delete_model_client.call(delete_model_srv))
        ROS_INFO_NAMED("INFO", "Successfully deleted %s", delete_model_srv.request.model_name.c_str());
    else
        ROS_INFO_NAMED("ERROR", "Could not delete %s", delete_model_srv.request.model_name.c_str());

    return true;
}


// Get object pose
geometry_msgs::Pose Robot::getModelPose(std::string model_name)
{
    geometry_msgs::Pose target_pose;
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = model_name; 

    if (this->get_model_state_client.call(srv))
    {
        ROS_INFO("Service Call Result: %d", srv.response.success);
        ROS_INFO("Service Call Status: %s", srv.response.status_message.c_str());
        target_pose = srv.response.pose;

    }
    else
    {
        ROS_ERROR("Failed to call service get_model_state");

        exit;
    }

    return target_pose;

}


// Set object pose
bool Robot::setModelPose(std::string model_name, geometry_msgs::Pose target_pose)
{
    gazebo_msgs::SetModelState set_model_state_srv;
    set_model_state_srv.request.model_state.model_name = model_name;
    set_model_state_srv.request.model_state.pose = target_pose;

    if (this->set_model_state_client.call(set_model_state_srv))
        ROS_INFO("INFO", "Successfully set model % pose", set_model_state_srv.request.model_state.model_name);
    else
        ROS_INFO("ERROR", "Could notset model % pose", set_model_state_srv.request.model_state.model_name);


    return true;
}



// Order service request callback
bool Robot::fulfillOrderCallback(std_srvs::SetBoolRequest  &req, std_srvs::SetBoolResponse &res)
{
    std::string order = (req.data) ? "red_box" : "blue_box";

    ROS_INFO_NAMED("SERVICE CALL INFO", "Received Request: %s", order.c_str());

    // Get model pose

    // geometry_msgs::Pose target_pose = this->workspace_pose;

    // // Change ee orientation to be facing down
    // target_pose.position.z -= 0.05;
    // target_pose.orientation.x = 0.5;
    // target_pose.orientation.y = 0.5;
    // target_pose.orientation.z = -0.5;
    // target_pose.orientation.w = 0.5;

    // // Create drop pose location and offset it
    // // geometry_msgs::Pose drop_pose = target_pose;
    // // drop_pose.position.y += 0.2;

    // Pick and Drop
    bool success = false;
    std::string obj_color;
    while (!success)
    {
        geometry_msgs::Pose target_pose = this->workspace_pose;
        // // Change ee orientation to be facing down
        target_pose.position.z -= 0.05;
        target_pose.orientation.x = 0.5;
        target_pose.orientation.y = 0.5;
        target_pose.orientation.z = -0.5;
        target_pose.orientation.w = 0.5;
        geometry_msgs::Pose drop_pose = target_pose;
        this->goToWorkspace();
        ROS_INFO("JUST ARRIVED TO WORKSPACE");
        obj_color = this->classifyObject();
        ROS_INFO("OBJECT CLASSIFIED");
        this->pickUp(target_pose, drop_pose);
        ROS_INFO("PICKED OBJECT UP");
        ROS_INFO("SUCCESFUL PICK UP!!");
        std::cout << "obj color: " << obj_color << " and order color: " << order << std::endl;
        if (obj_color == order)
        {
            ROS_INFO("ORDER AND CLASSIFICATION MATCH! BEGIN DISPATCH");
            this->dispatchOrder(target_pose, drop_pose);
            ROS_INFO("SUCCESFUL DISPATCH!!");
            success = true;
            break;
        }
        else
        {
            ROS_INFO("ORDER AND CLASSIFICATION DO NOT MATCH! OBJECT WILL BE DISCARDED");
            this->discardOrder(target_pose, drop_pose);
            ROS_INFO("SUCCESFUL DISCARD!!");
        }
            
    }
    
    ROS_INFO("ORDER FULFILLED! CLEANING SCENE UP");
    ros::Duration(2).sleep();

    // Delete and then respawn object
    this->deleteObject("red_box");
    this->deleteObject("blue_box");
    this->spawnObject((rand() % 2) ? "red_box" : "blue_box");



    // Respose
    res.success = true;
    res.message = ("%s order fulfilled!", order);


    return true;
}



// Classify using AWS Sagemaker (Cloud inference)
std::string Robot::classifyObject()
{
    std_srvs::Empty srv;
    std::string obj_color;

    bool res = this->aws_client.call(srv);
    std::cout << "AWS Response: " << res << std::endl;
    this->robot_node.getParam("/classification_result", obj_color);
    
    std::cout << "inference result: " << obj_color << std::endl;

    this->robot_node.setParam("/classification_result", "None");
    
    return obj_color;
}


// Local inference for test purposes
// std::string Robot::classifyObject()
// {
//     gazebo_msgs::GetModelState srv;
//     srv.request.model_name = "blue_box"; 

//     if (this->get_model_state_client.call(srv))
//         return "blue_box";
//     else
//         return "red_box";
    
// }