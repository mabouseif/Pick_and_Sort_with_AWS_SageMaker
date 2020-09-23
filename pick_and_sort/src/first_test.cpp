#include <ros/ros.h>
#include "robot.h"
// #include "image_converter.cpp"


int main(int argc, char** argv)
{

    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    // ImageConverter ic;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Initialize Robot
    Robot robot;

    // Clear and Spawn object
    robot.deleteObject("red_box");
    robot.deleteObject("blue_box");
    srand (time(NULL));
    robot.spawnObject((rand() % 2) ? "red_box" : "blue_box");

    // Go to init pose
    // robot.goToWorkspace();

    // Classify object
    // robot.classifyObject();


    // Press Ctrl+C to shut down
    ros::waitForShutdown();


    return 0;

}
