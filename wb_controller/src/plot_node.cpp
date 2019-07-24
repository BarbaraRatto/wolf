/**
 * @file plotnode.cpp
 * @author Gennaro Raiola, Michele Focchi
 * @date 12 June, 2019
 * @brief plot node.
 */

#include <stdio.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
// PluginLib
#include <pluginlib/class_list_macros.hpp>
// Ros control
#include <stdlib.h>
#include <wb_controller/utils.h>
#include <geometry_msgs/WrenchStamped.h>
#include <wb_controller/ContactForces.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

using namespace Eigen;
using namespace std;
using namespace wb_controller;

namespace wb_controller
{

/** @brief Visual tools */
rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

ros::Subscriber des_grf_subscriber_;
void callbackDesGRF(const wb_controller::ContactForces &msg);

bool init(ros::NodeHandle  & n)
{

    std::cout<<"Init plot node"<<std::endl;
    // Create subscribers

    des_grf_subscriber_ =  n.subscribe("des_grf", 1, callbackDesGRF);
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","des_grfs"));


    return true;
}

void callbackDesGRF(const wb_controller::ContactForces& msg)
{
   //ROS_INFO("I heard: [%s]", msg->data);

    Eigen::Affine3d pose;
    std::cout<< "force"<<std::endl;
    //Display an arrow along the x-axis of a pose.
    for(unsigned int i=0; i<msg.contact_forces.size();i++)
    {
            Vector3d force;
            force << msg.contact_forces[i].force.x,  msg.contact_forces[i].force.y, msg.contact_forces[i].force.z;
            double force_norm = force.norm();

            //find rotation matrix to align 1 0  0 to force direction
            Matrix3d R = Quaterniond().setFromTwoVectors(Vector3d::UnitX(),force/force_norm).toRotationMatrix();
            pose.linear() = R;
            pose.translation().x() = msg.contact_positions[i].x;
            pose.translation().y() = msg.contact_positions[i].y;
            pose.translation().z() = msg.contact_positions[i].z;

            visual_tools_->publishArrow(pose, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE, 1.0);
    }
    visual_tools_->trigger();

}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wb_controller");
  //put the node hanlde inside the specific plot_node name space rather than just the root "robot_name"
  //name space specified in the launch file
  ros::NodeHandle node("wb_controller");
  wb_controller::init(node);




  ROS_INFO("Ready to plot");
  ros::spin();


  return 0;
}


