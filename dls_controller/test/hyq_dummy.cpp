// ROS
#include <ros/ros.h>

// ros_control
#include <controller_manager/controller_manager.h>

#include "hyq_dummy.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hyq_dummy");
    ros::NodeHandle nh;

    HyqDummy robot;
    std::string joints_param_name = "dls_controller/joints";
    std::vector<std::string> joint_names;

    // Get joint names from the parameter server, using the controller config file
    //using namespace XmlRpc;
    //XmlRpcValue joint_names;
    if (!nh.getParam(joints_param_name, joint_names))
    {
        ROS_ERROR_STREAM_NAMED("hyq_dummy","No joints given (expected namespace: /" + joints_param_name + ").");
        return 1;
    }
    if (joint_names.size()==0)
    {
        ROS_ERROR_STREAM_NAMED("hyq_dummy","joints list empty.");
        return 1;
    }

    if(!robot.initializeInterfaces(joint_names))
    {
        ROS_ERROR_NAMED("hyq_dummy","Can not initialize the hardware interfaces.");
        return 1;
    }
    if(!robot.registerInterfaces())
    {
        ROS_ERROR_NAMED("hyq_dummy","Can not register the hardware interfaces.");
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
        ROS_INFO_STREAM("Running...");
        robot.read();
        ROS_INFO_STREAM("Read complete...");
        cm.update(robot.getTime(), robot.getPeriod());
        ROS_INFO_STREAM("Controller Manager update complete...");
        robot.write();
        ROS_INFO_STREAM("Write complete...");
        rate.sleep();
    }
    spinner.stop();

    return 0;
}
