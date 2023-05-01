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
  // Setup the interpolator
  trj_ = std::make_shared<wolf_controller::CartesianTrajectory>();

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
    ROS_WARN("No lambda1 gain given for task %s in the namespace: %s, using the default value loaded from the task",_task_id.c_str(),nh_.getNamespace().c_str());
    lambda1 = getLambda();
  }
  if (!nh_.getParam("gains/"+_task_id+"/weight" , weight))
  {
    ROS_WARN("No weight gain given for task %s in the namespace: %s, using the default value loaded from the task",_task_id.c_str(),nh_.getNamespace().c_str());
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

void Wrench::publish(const ros::Time& time)
{
  if(rt_pub_->trylock())
  {
    rt_pub_->msg_.header.frame_id = getBaseLink();
    rt_pub_->msg_.header.stamp = time;

    // ACTUAL VALUES
    //getActualPose(tmp_vector3d_);
    //// Pose - Translation
    //wolf_controller_utils::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.position_actual);
    //// Velocity reference
    //tmp_vector3d_ = getCachedVelocityReference();
    //wolf_controller_utils::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.velocity_reference);
    //
    //// REFERENCE VALUES
    //getReference(tmp_vector3d_);
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
    // Interpolation
    trj_->update(wolf_controller::_period);
    trj_->getReference(tmp_affine3d_,&tmp_vector6d_,&tmp_vector6d_1_);
    //setReference(tmp_affine3d_.translation(),tmp_vector6d_.head(3),tmp_vector6d_1_.head(3));
  }
  OpenSoT::tasks::force::Wrench::_update(x);
}

bool Wrench::reset()
{
  bool res = OpenSoT::tasks::force::Wrench::reset(); // Task's reset
  //getActualPose(tmp_vector3d_);
  //tmp_affine3d_ = Eigen::Affine3d::Identity();
  //tmp_affine3d_.translation() = tmp_vector3d_;
  //trj_->reset(tmp_affine3d_);
  return res;
}

void Wrench::referenceCallback(const wolf_msgs::WrenchTask::ConstPtr& msg)
{
  double period = wolf_controller::_period;

  if(last_time_ != 0.0)
    period = msg->header.stamp.toSec() - last_time_;

  Eigen::Affine3d pose_reference = Eigen::Affine3d::Identity();
  //pose_reference.translation().x() = msg->position_reference.x;
  //pose_reference.translation().y() = msg->position_reference.y;
  //pose_reference.translation().z() = msg->position_reference.z;
  trj_->setWayPoint(pose_reference,period);

  last_time_ = msg->header.stamp.toSec();
}


