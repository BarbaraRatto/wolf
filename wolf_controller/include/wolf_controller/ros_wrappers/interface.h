#ifndef ROS_WRAPPERS_INTERFACE_H
#define ROS_WRAPPERS_INTERFACE_H

// ROS
#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <interactive_markers/interactive_marker_server.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <wolf_controller/utils.h>

// STD
#include <atomic>

// WoLF
#include <wolf_controller/cartesian_trajectory.h>

class RosWrapperInterface
{

public:

  typedef std::shared_ptr<RosWrapperInterface> Ptr;

  RosWrapperInterface(){spinner_.reset(new ros::AsyncSpinner(1)); spinner_->start();}
  virtual ~RosWrapperInterface(){spinner_->stop();}
  virtual void publish(const ros::Time& /*time*/) = 0;

protected:

  std::shared_ptr<ros::AsyncSpinner> spinner_;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> server_;
};

template<class Msg_type>
class TaskRosWrapperInterface : public RosWrapperInterface
{

public:

  typedef std::shared_ptr<TaskRosWrapperInterface> Ptr;

  struct {
    std::atomic<bool> set_ext_lambda    = true;
    std::atomic<bool> set_ext_weight    = true;
    std::atomic<bool> set_ext_gains     = true;
    std::atomic<bool> set_ext_reference = false;
  } OPTIONS;

  TaskRosWrapperInterface(const std::string& task_name, ros::NodeHandle& nh)
  {
    task_name_ = task_name;
    nh_ = nh;
    rt_pub_.reset(new realtime_tools::RealtimePublisher<Msg_type>(nh_,task_name_, 4));
    ros::NodeHandle task_nh(nh_.getNamespace()+"/"+task_name_);
    server_.reset(new ddynamic_reconfigure::DDynamicReconfigure(task_nh));
  }

  virtual ~TaskRosWrapperInterface(){}

  virtual void loadParams() = 0;

  virtual void registerReconfigurableVariables() = 0;

  virtual void updateCost(const Eigen::VectorXd& x) = 0;

  void setLambda1(double value)    {buffer_lambda1_ = value;     ROS_INFO_STREAM(task_name_<<" - "<<"Set lambda1: "<<value);}
  void setLambda2(double value)    {buffer_lambda2_ = value;     ROS_INFO_STREAM(task_name_<<" - "<<"Set lambda2: "<<value);}
  void setWeightDiag(double value) {buffer_weight_diag_ = value; ROS_INFO_STREAM(task_name_<<" - "<<"Set weight diagonal: "<<value);}

  void setKpX(double value)     { buffer_kp_x_     = value; ROS_INFO_STREAM(task_name_<<" - "<<"Set Kp(0,0): "<<value); }
  void setKpY(double value)     { buffer_kp_y_     = value; ROS_INFO_STREAM(task_name_<<" - "<<"Set Kp(1,1): "<<value); }
  void setKpZ(double value)     { buffer_kp_z_     = value; ROS_INFO_STREAM(task_name_<<" - "<<"Set Kp(2,2): "<<value); }
  void setKpRoll(double value)  { buffer_kp_roll_  = value; ROS_INFO_STREAM(task_name_<<" - "<<"Set Kp(3,3): "<<value); }
  void setKpPitch(double value) { buffer_kp_pitch_ = value; ROS_INFO_STREAM(task_name_<<" - "<<"Set Kp(4,4): "<<value); }
  void setKpYaw(double value)   { buffer_kp_yaw_   = value; ROS_INFO_STREAM(task_name_<<" - "<<"Set Kp(5,5): "<<value); }

  void setKdX(double value)     { buffer_kd_x_     = value; ROS_INFO_STREAM(task_name_<<" - "<<"Set Kd(0,0): "<<value); }
  void setKdY(double value)     { buffer_kd_y_     = value; ROS_INFO_STREAM(task_name_<<" - "<<"Set Kd(1,1): "<<value); }
  void setKdZ(double value)     { buffer_kd_z_     = value; ROS_INFO_STREAM(task_name_<<" - "<<"Set Kd(2,2): "<<value); }
  void setKdRoll(double value)  { buffer_kd_roll_  = value; ROS_INFO_STREAM(task_name_<<" - "<<"Set Kd(3,3): "<<value); }
  void setKdPitch(double value) { buffer_kd_pitch_ = value; ROS_INFO_STREAM(task_name_<<" - "<<"Set Kd(4,4): "<<value); }
  void setKdYaw(double value)   { buffer_kd_yaw_   = value; ROS_INFO_STREAM(task_name_<<" - "<<"Set Kd(5,5): "<<value); }

protected:

  ros::NodeHandle       nh_;

  std::string           task_name_;

  Eigen::VectorXd       tmp_vectorXd_;
  Eigen::Affine3d       tmp_affine3d_;
  Eigen::Vector6d       tmp_vector6d_;
  Eigen::Vector3d       tmp_vector3d_;
  Eigen::Matrix6d       tmp_matrix6d_;
  Eigen::Matrix3d       tmp_matrix3d_;
  Eigen::Quaterniond    tmp_quaterniond_;

  realtime_tools::RealtimeBuffer<Eigen::Affine3d> buffer_pose_reference_;

  std::atomic<double> buffer_lambda1_;
  std::atomic<double> buffer_lambda2_;
  std::atomic<double> buffer_weight_diag_;

  std::atomic<double> buffer_kp_x_;
  std::atomic<double> buffer_kp_y_;
  std::atomic<double> buffer_kp_z_;
  std::atomic<double> buffer_kp_roll_;
  std::atomic<double> buffer_kp_pitch_;
  std::atomic<double> buffer_kp_yaw_;

  std::atomic<double> buffer_kd_x_;
  std::atomic<double> buffer_kd_y_;
  std::atomic<double> buffer_kd_z_;
  std::atomic<double> buffer_kd_roll_;
  std::atomic<double> buffer_kd_pitch_;
  std::atomic<double> buffer_kd_yaw_;

  std::shared_ptr<realtime_tools::RealtimePublisher<Msg_type>> rt_pub_;
  wolf_controller::CartesianTrajectory::Ptr trj_;

  double cost_;

};

#endif // ROS_WRAPPERS_INTERFACE_H

