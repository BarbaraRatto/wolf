// ROS
#include <ros/ros.h>
// Ros_control
#include <controller_manager/controller_manager.h>
#include "robot_dummy.h"

#define CLASS_NAME "RobotDummy"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_dummy");
    ros::NodeHandle nh;

    RobotDummy robot;
    std::string ns = ros::this_node::getNamespace();
    std::string joints_param_name = ns+"/wolf_controller/joints";
    std::vector<std::string> joint_names;

    if (!nh.getParam(joints_param_name, joint_names))
    {
        ROS_ERROR_STREAM_NAMED(CLASS_NAME,"No joints given (expected namespace: /" + joints_param_name + ").");
        return 1;
    }
    if (joint_names.size()==0)
    {
        ROS_ERROR_STREAM_NAMED(CLASS_NAME,"joints list empty.");
        return 1;
    }

    if(!robot.initializeInterfaces(joint_names))
    {
        ROS_ERROR_NAMED(CLASS_NAME,"Can not initialize the hardware interfaces.");
        return 1;
    }
    if(!robot.registerInterfaces())
    {
        ROS_ERROR_NAMED(CLASS_NAME,"Can not register the hardware interfaces.");
        return 1;
    }

    ROS_DEBUG_STREAM("period: " << robot.getPeriod().toSec());
    controller_manager::ControllerManager cm(&robot, nh);
    ROS_DEBUG_STREAM("Controller Manager created!");

    ros::Rate rate(1.0 / robot.getPeriod().toSec());
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok())
    {
        ROS_DEBUG_STREAM("Running...");
        robot.read();
        ROS_DEBUG_STREAM("Read complete...");
        cm.update(robot.getTime(), robot.getPeriod());
        ROS_DEBUG_STREAM("Controller Manager update complete...");
        robot.write();
        ROS_DEBUG_STREAM("Write complete...");
        rate.sleep();
    }
    spinner.stop();

    return 0;
}
