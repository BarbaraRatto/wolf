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
#include <wb_controller/CartesianTask.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace wb_controller
{

class ContactForcesVisualizer {

  public:

  ContactForcesVisualizer(ros::NodeHandle& nh)
  {
        cnt_ = 0;
        decimate_ = 10;

        subscriber_ = nh.subscribe("contact_forces", 1, &ContactForcesVisualizer::callbackContactForces, this);
        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","contact_forces_visual_marker"));
  }

  inline void callbackContactForces(const wb_controller::ContactForces &msg)
  {
    if(cnt_++%decimate_==0)
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

private:

  void createArrow(const geometry_msgs::Vector3& force, const geometry_msgs::Vector3& position,  rviz_visual_tools::colors color)
  {
      vector_(0) = force.x;
      vector_(1) = force.y;
      vector_(2) = force.z;
      norm_ = vector_.norm()+0.00001;

      //find rotation matrix to align 1 0  0 to force direction
      R_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(),vector_/norm_).toRotationMatrix();
      pose_.linear() = R_;
      pose_.translation().x() = position.x;
      pose_.translation().y() = position.y;
      pose_.translation().z() = position.z;
      visual_tools_->publishArrow(pose_, color, rviz_visual_tools::LARGE, norm_/500.0);
      visual_tools_->trigger();
  }

  unsigned int cnt_;
  unsigned int decimate_;
  Eigen::Isometry3d pose_;
  Eigen::Vector3d vector_;
  double norm_;
  Eigen::Matrix3d R_;
  ros::Subscriber subscriber_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

};

class CoMVisualizer {

  public:

  CoMVisualizer(ros::NodeHandle& nh)
  {
        cnt_ = 0;
        decimate_ = 10;

        subscriber_ = nh.subscribe("CoM", 1, &CoMVisualizer::callbackCoM, this);
        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","com_visual_marker"));
  }

  void callbackCoM(const wb_controller::CartesianTask& msg)
  {
    if(cnt_++%decimate_==0)
    {
        visual_tools_->deleteAllMarkers();

        createSphere(msg.pose_actual,rviz_visual_tools::GREEN);
        createSphere(msg.pose_reference,rviz_visual_tools::BLUE);
    }
  }

private:

  void createSphere(const geometry_msgs::Pose& pose, rviz_visual_tools::colors color)
  {
    pose_.linear() = Eigen::Matrix3d::Identity();
    pose_.translation().x() = pose.position.x;
    pose_.translation().y() = pose.position.y;
    pose_.translation().z() = pose.position.z;
    visual_tools_->publishSphere(pose_,color,rviz_visual_tools::XXLARGE);
    visual_tools_->trigger();
  }

  unsigned int cnt_;
  unsigned int decimate_;
  Eigen::Isometry3d pose_;
  ros::Subscriber subscriber_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

};



} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wb_controller");

  //put the node hanlde inside the specific plot_node name space rather than just the root "robot_name"
  //name space specified in the launch file

  ros::NodeHandle node("wb_controller");

  wb_controller::ContactForcesVisualizer cfv(node);
  wb_controller::CoMVisualizer comv(node);

  ros::spin();

  return 0;
}


