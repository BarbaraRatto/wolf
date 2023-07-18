/**
 * @file wrench.cpp
 * @author Gennaro Raiola
 * @date 25 April, 2023
 * @brief This file contains the wrench task wrapper for ROS
 */

// WoLF
#include <wolf_controller/ros_wrappers/wrench.h>

Wrench::Wrench(ros::NodeHandle& nh,
               const std::string& task_id,
               const std::string& distal_link,
               const std::string& base_link,
               OpenSoT::AffineHelper& wrench)
  :OpenSoT::tasks::force::Wrench(task_id,distal_link,base_link,wrench)
  ,TaskRosWrapperInterface<wolf_msgs::WrenchTask>(task_id,nh)
{
  tmp_vectorXd_.resize(6); // Wrench
  tmp_vectorXd_.setZero();

  // Initialize buffers
  buffer_reference_.initRT(tmp_vectorXd_);

  // Create the reference subscriber
  reference_sub_ = nh.subscribe("reference/"+_task_id, 1000, &Wrench::referenceCallback, this);
}

void Wrench::registerReconfigurableVariables()
{
  double lambda1 = getLambda();
  double weight  = getWeight()(0,0);
  ddr_server_->registerVariable<double>("set_lambda_1",    lambda1,     boost::bind(&TaskRosWrapperInterface::setLambda1,this,_1)    ,"set lambda 1"   ,0.0,1000.0);
  ddr_server_->registerVariable<double>("set_weight_diag", weight,      boost::bind(&TaskRosWrapperInterface::setWeightDiag,this,_1) ,"set weight diag",0.0,1000.0);
  ddr_server_->publishServicesTopics();
}

void Wrench::loadParams()
{

  double lambda1, weight;
  if (!nh_.getParam("gains/"+_task_id+"/lambda1" , lambda1))
  {
    ROS_DEBUG("No lambda1 gain given for task %s in the namespace: %s, using the default value loaded from the task",_task_id.c_str(),nh_.getNamespace().c_str());
    lambda1 = getLambda();
  }
  if (!nh_.getParam("gains/"+_task_id+"/weight" , weight))
  {
    ROS_DEBUG("No weight gain given for task %s in the namespace: %s, using the default value loaded from the task",_task_id.c_str(),nh_.getNamespace().c_str());
    weight = getWeight()(0,0);
  }
  // Check if the values are positive
  if(lambda1 < 0 || weight < 0)
    throw std::runtime_error("Lambda and weight must be positive!");

  buffer_lambda1_ = lambda1;
  buffer_weight_diag_ = weight;

  setLambda(lambda1);
  setWeight(weight);
}

void Wrench::updateCost(const Eigen::VectorXd& x)
{
  cost_ = computeCost(x);
}

void Wrench::publish(const ros::Time& time, const ros::Duration& /*period*/)
{
  if(rt_pub_->trylock())
  {
    rt_pub_->msg_.header.frame_id = getBaseLink();
    rt_pub_->msg_.header.stamp = time;

    // FIXME
    // ACTUAL VALUES
    //getActualPose(tmp_vector3d_);
    //// Pose - Translation
    //wolf_controller_utils::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.position_actual);
    //// Velocity reference
    //tmp_vector3d_ = getCachedVelocityReference();
    //wolf_controller_utils::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.velocity_reference);

    // REFERENCE VALUES
    //getReference(tmp_vectorXd_);
    //// Pose - Translation
    //wolf_controller_utils::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.position_reference);

    // COST
    rt_pub_->msg_.cost = cost_;

    rt_pub_->unlockAndPublish();
  }
}

void Wrench::_update(const Eigen::VectorXd& x)
{
  if(OPTIONS.set_ext_lambda)
    setLambda(buffer_lambda1_);
  if(OPTIONS.set_ext_weight)
    setWeight(buffer_weight_diag_);
  if(OPTIONS.set_ext_reference)
  {
    // Set external reference
    setReference(*buffer_reference_.readFromRT());
  }
  OpenSoT::tasks::force::Wrench::_update(x);
}

bool Wrench::reset()
{
  //bool res = OpenSoT::tasks::force::Wrench::reset(); // Task's reset (FIXME it is not implemented in OpenSoT)
  bool res = true;
  getReference(tmp_vectorXd_);
  buffer_reference_.initRT(tmp_vectorXd_);
  //getActualPose(tmp_vector3d_);
  //tmp_affine3d_ = Eigen::Affine3d::Identity();
  //tmp_affine3d_.translation() = tmp_vector3d_;
  //trj_->reset(tmp_affine3d_);
  return res;
}

void Wrench::referenceCallback(const wolf_msgs::Wrench::ConstPtr& msg)
{
  double period = wolf_controller::_period;

  if(last_time_ != 0.0)
    period = msg->header.stamp.toSec() - last_time_;

  Eigen::Vector6d reference = Eigen::Vector6d::Zero();
  reference(0) = msg->wrench.force.x;
  reference(1) = msg->wrench.force.y;
  reference(2) = msg->wrench.force.z;
  buffer_reference_.writeFromNonRT(reference);

  last_time_ = msg->header.stamp.toSec();
}
