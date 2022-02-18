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
#include <wolf_controller/utils.h>
#include <wolf_controller/ContactForces.h>
#include <wolf_controller/CapturePoint.h>
#include <wolf_controller/CartesianTask.h>
#include <wolf_controller/ComTask.h>
#include <wolf_controller/FrictionCones.h>
#include <wolf_controller/TerrainEstimation.h>
#include <wolf_controller/FootHolds.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <mutex>

/*
 * BLUE: DESIRED
 * GREEN: ACTUAL
 *
 */
namespace rviz_visual_tools {

class VisualTools : public RvizVisualTools
{

public:

  typedef std::shared_ptr<VisualTools> Ptr;

  VisualTools(std::string base_frame, std::string marker_topic = RVIZ_MARKER_TOPIC, ros::NodeHandle nh = ros::NodeHandle("~"))
    :RvizVisualTools(base_frame,marker_topic,nh) {}

  bool publishFrictionCone(const Eigen::Vector3d& origin, const double& height, const Eigen::Vector3d& normal, const double& friction_coeff, colors color)
  {
    double radius = friction_coeff * height;
    Eigen::Vector3d tail_end = origin + normal*height;

    // Set the frame ID and timestamp.
    arrow_marker_.header.stamp = ros::Time::now();
    arrow_marker_.header.frame_id = base_frame_;
    arrow_marker_.type = visualization_msgs::Marker::ARROW;
    arrow_marker_.action = visualization_msgs::Marker::ADD;
    arrow_marker_.id++;

    arrow_marker_.points.clear();
    geometry_msgs::Point p, start;
    p.x = tail_end(0);
    p.y = tail_end(1);
    p.z = tail_end(2);
    start.x = origin(0);
    start.y = origin(1);
    start.z = origin(2);
    arrow_marker_.pose.orientation.x = 0;
    arrow_marker_.pose.orientation.y = 0;
    arrow_marker_.pose.orientation.z = 0;
    arrow_marker_.pose.orientation.w = 1;
    arrow_marker_.points.resize(0);
    arrow_marker_.points.push_back(p);
    arrow_marker_.points.push_back(start);
    arrow_marker_.color = getColor(color);
    arrow_marker_.scale.x = 0.001;
    arrow_marker_.scale.y = 2 * radius;
    arrow_marker_.scale.z = height;

    // Helper for publishing rviz markers
    return publishMarker(arrow_marker_);
  }

};

}


namespace wolf_controller
{

static std::mutex _mtx;

template <class msg_t>
class Visualizer
{

public:

  Visualizer(ros::NodeHandle& nh, const std::string& topic_name, const std::string& base_frame = "world")
  {
    cnt_ = 0;
    decimate_ = 10;
    subscriber_ = nh.subscribe(topic_name, 1, &Visualizer::callback, this);
    visual_tools_.reset(new rviz_visual_tools::VisualTools(base_frame,topic_name+"_visual_marker"));
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
    q_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(),vector_/norm_);
    q_.normalize();
    R_ = q_.toRotationMatrix();
    pose_.linear() = R_;
    pose_.translation().x() = position.x;
    pose_.translation().y() = position.y;
    pose_.translation().z() = position.z;
    visual_tools_->publishCone(pose_,angle,rviz_visual_tools::colors::LIME_GREEN,0.05);
    visual_tools_->trigger();
  }

  // FRICTION CONE
  void createFrictionCone(const geometry_msgs::Vector3& normal, const geometry_msgs::Vector3& position, const double& friction_coeff)
  {
    vector_(0) = normal.x;
    vector_(1) = normal.y;
    vector_(2) = normal.z;
    norm_ = vector_.norm()+0.00001;
    q_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(),vector_/norm_);
    q_.normalize();
    R_ = q_.toRotationMatrix();
    pose_.linear() = R_;
    pose_.translation().x() = position.x;
    pose_.translation().y() = position.y;
    pose_.translation().z() = position.z;
    visual_tools_->publishFrictionCone(pose_.translation(),0.05,vector_,friction_coeff,rviz_visual_tools::colors::LIME_GREEN);
    visual_tools_->trigger();
  }

  // PLANE
  void createPlane(const geometry_msgs::Vector3& normal, const geometry_msgs::Vector3& position)
  {
    vector_(0) = normal.x;
    vector_(1) = normal.y;
    vector_(2) = normal.z;
    norm_ = vector_.norm()+0.00001;
    q_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(),vector_/norm_);
    q_.normalize();
    R_ = q_.toRotationMatrix();
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
    q_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(),vector_/norm_);
    q_.normalize();
    R_ = q_.toRotationMatrix();
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
    q_ = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(),vector_/norm_);
    q_.normalize();
    R_ = q_.toRotationMatrix();
    pose_.linear() = R_;
    pose_.translation().x() = origin.x;
    pose_.translation().y() = origin.y;
    pose_.translation().z() = origin.z;
    visual_tools_->publishArrow(pose_, color, rviz_visual_tools::LARGE, norm_/scale);
    visual_tools_->trigger();
  }

  // POLYGON
  void createPolygon(const geometry_msgs::Polygon& poly, rviz_visual_tools::colors color)
  {
    visual_tools_->publishPolygon(poly,color,rviz_visual_tools::LARGE);
    visual_tools_->trigger();
  }

  // SPHERE
  void createSphere(const geometry_msgs::Point& origin, rviz_visual_tools::colors color)
  {
    R_.setIdentity();
    pose_.linear() = R_;
    pose_.translation().x() = origin.x;
    pose_.translation().y() = origin.y;
    pose_.translation().z() = origin.z;
    visual_tools_->publishSphere(pose_,color,rviz_visual_tools::XXLARGE);
    visual_tools_->trigger();
  }

  void createSphere(const geometry_msgs::Vector3& origin, rviz_visual_tools::colors color)
  {
    R_.setIdentity();
    pose_.linear() = R_;
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
  Eigen::Quaterniond q_;
  ros::Subscriber subscriber_;
  rviz_visual_tools::VisualTools::Ptr visual_tools_;
};


class FrictionConesVisualizer : public Visualizer<wolf_controller::FrictionCones>
{

public:

  typedef std::shared_ptr<FrictionConesVisualizer> Ptr;

  FrictionConesVisualizer(ros::NodeHandle& nh, const std::string& topic_name)
    :Visualizer<wolf_controller::FrictionCones>(nh,topic_name)
  {
  }

  virtual ~FrictionConesVisualizer() {}

protected:

  virtual void callback(const wolf_controller::FrictionCones &msg) override
  {
    if(cnt_++%decimate_==0 && _mtx.try_lock())
    {
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg.header.frame_id);
      for(unsigned int i=0; i<msg.foot_positions.size();i++)
        createFrictionCone(msg.cone_axis[i],msg.foot_positions[i],static_cast<double>(msg.mus[i].data));
      //createCone(msg.cone_axis[i],msg.foot_positions[i],static_cast<double>(std::atan(msg.mus[i].data)));
      _mtx.unlock();
    }
  }
};

class TerrainEstimationVisualizer : public Visualizer<wolf_controller::TerrainEstimation>
{

public:

  typedef std::shared_ptr<TerrainEstimationVisualizer> Ptr;

  TerrainEstimationVisualizer(ros::NodeHandle& nh, const std::string& topic_name)
    :Visualizer<wolf_controller::TerrainEstimation>(nh,topic_name)
  {
  }

  virtual ~TerrainEstimationVisualizer() {}

protected:
  virtual void callback(const wolf_controller::TerrainEstimation &msg) override
  {
    if(cnt_++%decimate_==0 && _mtx.try_lock())
    {
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg.header.frame_id);
      createPlane(msg.terrain_normal,msg.central_point);
      createArrow(msg.terrain_normal,msg.central_point,rviz_visual_tools::CYAN,10.0);
      _mtx.unlock();
    }
  }
};

class ContactForcesVisualizer : public Visualizer<wolf_controller::ContactForces>
{

public:

  ContactForcesVisualizer(ros::NodeHandle& nh, const std::string& topic_name)
    :Visualizer<wolf_controller::ContactForces>(nh,topic_name)
  {
  }

  virtual ~ContactForcesVisualizer() {}

protected:

  virtual void callback(const wolf_controller::ContactForces &msg) override
  {
    if(cnt_++%decimate_==0 && _mtx.try_lock())
    {
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg.header.frame_id);
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

class CoMVisualizer : public Visualizer<wolf_controller::ComTask>
{

public:

  CoMVisualizer(ros::NodeHandle& nh, const std::string& topic_name)
    :Visualizer<wolf_controller::ComTask>(nh,topic_name)
  {
  }

  virtual ~CoMVisualizer() {}

protected:

  virtual void callback(const wolf_controller::ComTask& msg) override
  {
    if(cnt_++%decimate_==0 && _mtx.try_lock())
    {
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg.header.frame_id);
      wolf_controller::ComTask com_projection = msg;
      com_projection.position_actual.z = 0.0;
      createSphere(com_projection.position_actual,rviz_visual_tools::RED);
      createSphere(msg.position_actual,rviz_visual_tools::GREEN);
      createArrow(msg.velocity_reference,msg.position_actual,rviz_visual_tools::BLUE,1.0);
      _mtx.unlock();
    }
  }
};

class CapturePointVisualizer : public Visualizer<wolf_controller::CapturePoint>
{

public:

  CapturePointVisualizer(ros::NodeHandle& nh, const std::string& topic_name)
    :Visualizer<wolf_controller::CapturePoint>(nh,topic_name)
  {
  }

  virtual ~CapturePointVisualizer() {}

protected:

  virtual void callback(const wolf_controller::CapturePoint& msg) override
  {
    if(cnt_++%decimate_==0 && _mtx.try_lock())
    {
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg.header.frame_id);
      wolf_controller::CapturePoint data = msg;
      createSphere(data.capture_point,rviz_visual_tools::RED);
      createSphere(data.com,rviz_visual_tools::GREEN);
      createPolygon(data.support_polygon,rviz_visual_tools::GREEN);
      _mtx.unlock();
    }
  }
};

class FootHoldsVisualizer : public Visualizer<wolf_controller::FootHolds>
{

public:

  FootHoldsVisualizer(ros::NodeHandle& nh, const std::string& topic_name)
    :Visualizer<wolf_controller::FootHolds>(nh,topic_name)
  {
  }

  virtual ~FootHoldsVisualizer() {}

protected:
  virtual void callback(const wolf_controller::FootHolds& msg) override
  {
    if(cnt_++%decimate_==0 && _mtx.try_lock())
    {
      visual_tools_->deleteAllMarkers();
      visual_tools_->setBaseFrame(msg.header.frame_id);
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
  ros::init(argc, argv, "wolf_controller");

  //put the node hanlde inside the specific plot_node name space rather than just the root "robot_name"
  //name space specified in the launch file

  ros::NodeHandle node("wolf_controller");

  wolf_controller::ContactForcesVisualizer cfv(node,"contact_forces");
  wolf_controller::CoMVisualizer comv(node,"CoM");
  wolf_controller::FootHoldsVisualizer fhv(node,"foot_holds");
  wolf_controller::TerrainEstimationVisualizer tev(node,"terrain_estimation");
  wolf_controller::FrictionConesVisualizer fcv(node,"friction_cones");
  wolf_controller::CapturePointVisualizer cpv(node,"capture_point");

  ros::spin();

  return 0;
}


