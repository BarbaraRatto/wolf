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
#include <mutex>

/*
 * BLUE: DESIRED
 * GREEN: ACTUAL
 *
 */

// Override
bool rviz_visual_tools::RvizVisualTools::publishCone(const geometry_msgs::Pose& pose, double angle, colors color, double scale)
{
  triangle_marker_.header.stamp = ros::Time::now();
  triangle_marker_.id++;

  triangle_marker_.color = getColor(color);
  triangle_marker_.pose = pose;

  geometry_msgs::Point p[3];
  static const double DELTA_THETA = M_PI / 16.0;
  double theta = 0;

  triangle_marker_.points.clear();
  for (std::size_t i = 0; i < 32; i++)
  {
    p[0].x = 0;
    p[0].y = 0;
    p[0].z = 0;

    p[1].x = scale;
    p[1].y = scale * cos(theta) / angle;
    p[1].z = scale * sin(theta) / angle;

    p[2].x = scale;
    p[2].y = scale * cos(theta + DELTA_THETA) / angle;
    p[2].z = scale * sin(theta + DELTA_THETA) / angle;

    triangle_marker_.points.push_back(p[0]);
    triangle_marker_.points.push_back(p[1]);
    triangle_marker_.points.push_back(p[2]);

    theta += DELTA_THETA;
  }

  triangle_marker_.scale.x = 1.0;
  triangle_marker_.scale.y = 1.0;
  triangle_marker_.scale.z = 1.0;

  return publishMarker(triangle_marker_);
}

namespace wb_controller
{

static std::mutex _mtx;

template <class msg_t>
class Visualizer
{

public:

  Visualizer(ros::NodeHandle& nh, const std::string& topic_name)
  {
    cnt_ = 0;
    decimate_ = 10;
    subscriber_ = nh.subscribe(topic_name, 1, &Visualizer::callback, this);
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world",topic_name+"_visual_marker"));
  }

protected:

  virtual void callback(const msg_t& msg) = 0;

  // CONE
  void createCone(const geometry_msgs::Vector3& normal, const geometry_msgs::Vector3& position, const double& angle = M_PI)
  {
      vector_(0) = normal.x;
      vector_(1) = normal.y;
      vector_(2) = normal.z;
      norm_ = vector_.norm()+0.00001;

      R_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(),vector_/norm_).toRotationMatrix();
      pose_.linear() = R_;
      pose_.translation().x() = position.x;
      pose_.translation().y() = position.y;
      pose_.translation().z() = position.z;


      visual_tools_->publishCone(pose_,angle,rviz_visual_tools::colors::LIME_GREEN,0.05);
      visual_tools_->trigger();
  }

  // PLANE
  void createPlane(const geometry_msgs::Vector3& normal, const geometry_msgs::Vector3& position)
  {
      vector_(0) = normal.x;
      vector_(1) = normal.y;
      vector_(2) = normal.z;
      norm_ = vector_.norm()+0.00001;

      R_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(),vector_/norm_).toRotationMatrix();
      pose_.linear() = R_;
      pose_.translation().x() = position.x;
      pose_.translation().y() = position.y;
      pose_.translation().z() = position.z;

      visual_tools_->publishYZPlane(pose_);
      visual_tools_->trigger();
  }

  // ARROW
  void createArrow(const geometry_msgs::Vector3& vector, const geometry_msgs::Vector3& origin,  rviz_visual_tools::colors color, double scale = 500.0)
  {
      vector_(0) = vector.x;
      vector_(1) = vector.y;
      vector_(2) = vector.z;
      norm_ = vector_.norm()+0.00001;

      //find rotation matrix to align 1 0  0 to force direction
      R_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(),vector_/norm_).toRotationMatrix();
      pose_.linear() = R_;
      pose_.translation().x() = origin.x;
      pose_.translation().y() = origin.y;
      pose_.translation().z() = origin.z;
      visual_tools_->publishArrow(pose_, color, rviz_visual_tools::LARGE, norm_/scale);
      visual_tools_->trigger();
  }

  void createArrow(const geometry_msgs::Vector3& vector, const geometry_msgs::Point& origin,  rviz_visual_tools::colors color, double scale = 500.0)
  {
      vector_(0) = vector.x;
      vector_(1) = vector.y;
      vector_(2) = vector.z;
      norm_ = vector_.norm()+0.00001;

      //find rotation matrix to align 1 0  0 to force direction
      R_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(),vector_/norm_).toRotationMatrix();
      pose_.linear() = R_;
      pose_.translation().x() = origin.x;
      pose_.translation().y() = origin.y;
      pose_.translation().z() = origin.z;
      visual_tools_->publishArrow(pose_, color, rviz_visual_tools::LARGE, norm_/scale);
      visual_tools_->trigger();
  }

  // SPHERE
  void createSphere(const geometry_msgs::Point& origin, rviz_visual_tools::colors color)
  {
    pose_.translation().x() = origin.x;
    pose_.translation().y() = origin.y;
    pose_.translation().z() = origin.z;
    visual_tools_->publishSphere(pose_,color,rviz_visual_tools::XXLARGE);
    visual_tools_->trigger();
  }

  void createSphere(const geometry_msgs::Vector3& origin, rviz_visual_tools::colors color)
  {
    pose_.translation().x() = origin.x;
    pose_.translation().y() = origin.y;
    pose_.translation().z() = origin.z;
    visual_tools_->publishSphere(pose_,color,rviz_visual_tools::XXLARGE);
    visual_tools_->trigger();
  }

  long long cnt_;
  unsigned int decimate_;
  Eigen::Isometry3d pose_;
  Eigen::Vector3d vector_;
  double norm_;
  Eigen::Matrix3d R_;
  ros::Subscriber subscriber_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
};


class FrictionConesVisualizer : public Visualizer<wb_controller::FrictionCones>
{

public:

  typedef std::shared_ptr<FrictionConesVisualizer> Ptr;

  FrictionConesVisualizer(ros::NodeHandle& nh, const std::string& topic_name)
    :Visualizer<wb_controller::FrictionCones>(nh,topic_name)
  {
  }

  virtual ~FrictionConesVisualizer() {}

protected:

  virtual void callback(const wb_controller::FrictionCones &msg) override
  {
    if(cnt_++%decimate_==0 && _mtx.try_lock())
    {
        visual_tools_->deleteAllMarkers();
        for(unsigned int i=0; i<msg.foot_positions.size();i++)
           createCone(msg.cone_axis[i],msg.foot_positions[i],static_cast<double>(std::atan(msg.mus[i].data)));
        _mtx.unlock();
    }
  }
};

class TerrainEstimationVisualizer : public Visualizer<wb_controller::TerrainEstimation>
{

public:

  typedef std::shared_ptr<TerrainEstimationVisualizer> Ptr;

  TerrainEstimationVisualizer(ros::NodeHandle& nh, const std::string& topic_name)
    :Visualizer<wb_controller::TerrainEstimation>(nh,topic_name)
  {
  }

  virtual ~TerrainEstimationVisualizer() {}

protected:
  virtual void callback(const wb_controller::TerrainEstimation &msg) override
  {
    if(cnt_++%decimate_==0 && _mtx.try_lock())
    {
        visual_tools_->deleteAllMarkers();
        createPlane(msg.terrain_normal,msg.central_point);
        createArrow(msg.terrain_normal,msg.central_point,rviz_visual_tools::CYAN,10.0);
        _mtx.unlock();
    }
  }
};

class ContactForcesVisualizer : public Visualizer<wb_controller::ContactForces>
{

public:

  ContactForcesVisualizer(ros::NodeHandle& nh, const std::string& topic_name)
    :Visualizer<wb_controller::ContactForces>(nh,topic_name)
  {
  }

  virtual ~ContactForcesVisualizer() {}

protected:

  virtual void callback(const wb_controller::ContactForces &msg) override
  {
    if(cnt_++%decimate_==0 && _mtx.try_lock())
    {
        visual_tools_->deleteAllMarkers();
        //Display an arrow along the x-axis of a pose.
        for(unsigned int i=0; i<msg.contact.size();i++)
        {
           createArrow(msg.des_contact_forces[i].force,msg.contact_positions[i],rviz_visual_tools::BLUE);
           createArrow(msg.contact_forces[i].force,msg.contact_positions[i],rviz_visual_tools::GREEN);
           _mtx.unlock();
        }
    }
  }
};

class CoMVisualizer : public Visualizer<wb_controller::CartesianTask>
{

  public:

  CoMVisualizer(ros::NodeHandle& nh, const std::string& topic_name)
    :Visualizer<wb_controller::CartesianTask>(nh,topic_name)
  {
  }

  virtual ~CoMVisualizer() {}

protected:

  virtual void callback(const wb_controller::CartesianTask& msg) override
  {
    if(cnt_++%decimate_==0 && _mtx.try_lock())
    {
        visual_tools_->deleteAllMarkers();
        createSphere(msg.pose_actual.position,rviz_visual_tools::GREEN);
        createArrow(msg.twist_reference.linear,msg.pose_actual.position,rviz_visual_tools::BLUE,1.0);
        _mtx.unlock();
    }
  }
};

class FootHoldsVisualizer : public Visualizer<wb_controller::FootHolds>
{

  public:

  FootHoldsVisualizer(ros::NodeHandle& nh, const std::string& topic_name)
    :Visualizer<wb_controller::FootHolds>(nh,topic_name)
  {
  }

  virtual ~FootHoldsVisualizer() {}

protected:
  virtual void callback(const wb_controller::FootHolds& msg) override
  {
    if(cnt_++%decimate_==0 && _mtx.try_lock())
    {
        visual_tools_->deleteAllMarkers();
        for(unsigned int i=0;i<msg.name.size();i++)
        {
          //createSphere(msg.desired_foothold[i],rviz_visual_tools::BLUE);
          createSphere(msg.virtual_foothold[i],rviz_visual_tools::RED);
          _mtx.unlock();
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

  wb_controller::ContactForcesVisualizer cfv(node,"contact_forces");
  wb_controller::CoMVisualizer comv(node,"CoM");
  wb_controller::FootHoldsVisualizer fhv(node,"foot_holds");
  wb_controller::TerrainEstimationVisualizer tev(node,"terrain_estimation");
  wb_controller::FrictionConesVisualizer fcv(node,"friction_cones");

  ros::spin();

  return 0;
}


