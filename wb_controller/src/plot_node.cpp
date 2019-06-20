/**
 * @file plot_node.cpp
 * @author Gennaro Raiola, Michele Focchi
 * @date 12 June, 2019
 * @brief plot node.
 */

#include <stdio.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <geometry_msgs/WrenchStamped.h>
#include <wb_controller/utils.h>
#include <wb_controller/ContactForces.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace wb_controller
{

/** @brief Visual tools */
rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

ros::Subscriber des_grf_subscriber_;
void callbackGRF(const wb_controller::ContactForces &msg);
unsigned int decimate = 10;
unsigned long long cnt = 0;
Eigen::Affine3d pose;
Eigen::Vector3d vector;
double norm;
Eigen::Matrix3d R;

bool init(ros::NodeHandle& nh)
{

    ROS_INFO("Init");

    // Create subscribers
    des_grf_subscriber_ = nh.subscribe("grf", 1, callbackGRF);
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","grfs_visual_marker"));

    return true;
}

void createArrow(const geometry_msgs::Vector3& force, const geometry_msgs::Vector3& position,  rviz_visual_tools::colors color)
{
    vector(0) = force.x;
    vector(1) = force.y;
    vector(2) = force.z;
    norm = vector.norm();

    //find rotation matrix to align 1 0  0 to force direction
    R = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(),vector/norm).toRotationMatrix();
    pose.linear() = R;
    pose.translation().x() = position.x;
    pose.translation().y() = position.y;
    pose.translation().z() = position.z;

    visual_tools_->publishArrow(pose, color, rviz_visual_tools::LARGE, 1.0);
    visual_tools_->trigger();
}

void callbackGRF(const wb_controller::ContactForces& msg)
{
    if(cnt++%decimate==0)
    {
        visual_tools_->deleteAllMarkers();

        //Display an arrow along the x-axis of a pose.
        for(unsigned int i=0; i<msg.contact.size();i++)
        {
                createArrow(msg.des_contact_forces[i].force,msg.contact_positions[i],rviz_visual_tools::BLUE);
                createArrow(msg.contact_forces[i].force,msg.contact_positions[i],rviz_visual_tools::GREEN);
        }
    }

}

} // namespace

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


