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
#include <wb_controller/FrictionCones.h>
#include <wb_controller/TerrainEstimation.h>
#include <wb_controller/FootHolds.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace wb_controller
{

class FrictionConesVisualizer
{

public:

  typedef std::shared_ptr<FrictionConesVisualizer> Ptr;

  FrictionConesVisualizer(ros::NodeHandle& nh)
  {
    cnt_ = 0;
    decimate_ = 10;

    subscriber_ = nh.subscribe("friction_cones", 1, &FrictionConesVisualizer::callbackFrictionCones, this);
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","friction_cones_visual_marker"));
  }

  inline void callbackFrictionCones(const wb_controller::FrictionCones &msg)
  {
    if(cnt_++%decimate_==0)
    {
        visual_tools_->deleteAllMarkers();

        for(unsigned int i=0; i<msg.foot_positions.size();i++)
           createCone(msg.cone_axis[i],msg.foot_positions[i]);
    }
  }

private:

  void createCone(const geometry_msgs::Vector3& normal, const geometry_msgs::Vector3& position)
  {
      vector_(0) = normal.x;
      vector_(1) = normal.y;
      vector_(2) = normal.z;
      norm_ = vector_.norm();

      R_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(),vector_/norm_).toRotationMatrix();
      pose_.linear() = R_;
      pose_.translation().x() = position.x;
      pose_.translation().y() = position.y;
      pose_.translation().z() = position.z;

      visual_tools_->publishCone(pose_,M_PI);
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

class TerrainEstimationVisualizer
{

public:

  typedef std::shared_ptr<TerrainEstimationVisualizer> Ptr;

  TerrainEstimationVisualizer(ros::NodeHandle& nh)
  {
    cnt_ = 0;
    decimate_ = 10;

    subscriber_ = nh.subscribe("terrain_estimation", 1, &TerrainEstimationVisualizer::callbackTerrainEstimation, this);
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","terrain_estimation_visual_marker"));
  }

  inline void callbackTerrainEstimation(const wb_controller::TerrainEstimation &msg)
  {
    if(cnt_++%decimate_==0)
    {
        visual_tools_->deleteAllMarkers();
        createPlane(msg.terrain_normal,msg.central_point);
    }
  }

private:

  void createPlane(const geometry_msgs::Vector3& normal, const geometry_msgs::Vector3& position)
  {
      vector_(0) = normal.x;
      vector_(1) = normal.y;
      vector_(2) = normal.z;
      norm_ = vector_.norm();

      R_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(),vector_/norm_).toRotationMatrix();
      pose_.linear() = R_;
      pose_.translation().x() = position.x;
      pose_.translation().y() = position.y;
      pose_.translation().z() = position.z;

      visual_tools_->publishYZPlane(pose_);
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

template<class msg_t>
class SphereVisualizer {

public:

  SphereVisualizer(ros::NodeHandle& nh, const std::string& topic_name, const std::string& marker_topic, const std::string& frame="world")
  {
    cnt_ = 0;
    decimate_ = 10;
    origin_ = Eigen::Isometry3d::Identity();

    subscriber_ = nh.subscribe(topic_name, 1, &SphereVisualizer::callback, this);
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(frame,marker_topic,topic_name));
  }

  virtual ~SphereVisualizer() {}

  virtual void callback(const msg_t& msg) = 0;

protected:

  void createSphere(const geometry_msgs::Point& origin, rviz_visual_tools::colors color)
  {
    origin_.translation().x() = origin.x;
    origin_.translation().y() = origin.y;
    origin_.translation().z() = origin.z;
    visual_tools_->publishSphere(origin_,color,rviz_visual_tools::XXLARGE);
    visual_tools_->trigger();
  }

  void createSphere(const geometry_msgs::Vector3& origin, rviz_visual_tools::colors color)
  {
    origin_.translation().x() = origin.x;
    origin_.translation().y() = origin.y;
    origin_.translation().z() = origin.z;
    visual_tools_->publishSphere(origin_,color,rviz_visual_tools::XXLARGE);
    visual_tools_->trigger();
  }

  Eigen::Isometry3d origin_;
  unsigned int cnt_;
  unsigned int decimate_;
  ros::Subscriber subscriber_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

};

class CoMVisualizer : public SphereVisualizer<wb_controller::CartesianTask> {

  public:

  CoMVisualizer(ros::NodeHandle& nh)
    :SphereVisualizer<wb_controller::CartesianTask>(nh,"CoM","com_visual_marker")
  {
  }

  ~CoMVisualizer() {}

  virtual void callback(const wb_controller::CartesianTask& msg)
  {
    if(cnt_++%decimate_==0)
    {
        visual_tools_->deleteAllMarkers();
        SphereVisualizer::createSphere(msg.pose_actual.position,rviz_visual_tools::GREEN);
        //SphereVisualizer::createSphere(msg.pose_reference.position,rviz_visual_tools::BLUE);
    }
  }

};

class FootHoldsVisualizer : public SphereVisualizer<wb_controller::FootHolds> {

  public:

  FootHoldsVisualizer(ros::NodeHandle& nh)
    :SphereVisualizer<wb_controller::FootHolds>(nh,"foot_holds","footholds_visual_marker","base_link")
  {
  }

  ~FootHoldsVisualizer() {}

  virtual void callback(const wb_controller::FootHolds& msg)
  {
    if(cnt_++%decimate_==0)
    {
        visual_tools_->deleteAllMarkers();
        for(unsigned int i=0;i<msg.name.size();i++)
        {
          //createSphere(msg.desired_foothold[i],rviz_visual_tools::BLUE);
          createSphere(msg.virtual_foothold[i],rviz_visual_tools::RED);
        }
    }
  }

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
  wb_controller::FootHoldsVisualizer fhv(node);
  wb_controller::TerrainEstimationVisualizer tev(node);

  ros::spin();

  return 0;
}


