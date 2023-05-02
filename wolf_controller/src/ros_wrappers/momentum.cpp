/**
 * @file momentum.cpp
 * @author Gennaro Raiola
 * @date 24 April, 2023
 * @brief This file contains the momentum task wrapper for ROS
 */

// WoLF
#include <wolf_controller/ros_wrappers/momentum.h>

AngularMomentum::AngularMomentum(ros::NodeHandle& nh, XBot::ModelInterface& robot, const OpenSoT::AffineHelper& qddot)
  :OpenSoT::tasks::acceleration::AngularMomentum(robot,qddot)
  ,TaskRosWrapperInterface<wolf_msgs::CartesianTask>(_task_id,nh)
{
}

void AngularMomentum::registerReconfigurableVariables()
{
  double lambda1 = getLambda();
  double weight  = getWeight()(0,0);
  ddr_server_->registerVariable<double>("set_lambda_1",    lambda1,     boost::bind(&TaskRosWrapperInterface::setLambda1,this,_1)    ,"set lambda 1"   ,0.0,1000.0);
  ddr_server_->registerVariable<double>("set_weight_diag", weight,      boost::bind(&TaskRosWrapperInterface::setWeightDiag,this,_1) ,"set weight diag",0.0,1000.0);
  Eigen::Matrix3d K = getMomentumGain();
  ddr_server_->registerVariable<double>("K_roll",    K(0,0), boost::bind(&TaskRosWrapperInterface::setKpRoll, this,_1)    ,"K(0,0)", 0.0, 1000.0);
  ddr_server_->registerVariable<double>("K_pitch",   K(1,1), boost::bind(&TaskRosWrapperInterface::setKpPitch,this,_1)    ,"K(1,1)", 0.0, 1000.0);
  ddr_server_->registerVariable<double>("K_yaw",     K(2,2), boost::bind(&TaskRosWrapperInterface::setKpYaw,  this,_1)    ,"K(2,2)", 0.0, 1000.0);
  ddr_server_->publishServicesTopics();
}

void AngularMomentum::loadParams()
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

  // Load params
  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
  bool use_identity = false;
  for(unsigned int i=0; i<wolf_controller::_rpy.size(); i++)
  {
    if (!nh_.getParam("gains/"+_task_id+"/K/" + wolf_controller::_rpy[i] , K(i,i)))
    {
      ROS_WARN("No Kp.%s gain given for task %s in the namespace: %s, using an identity matrix. ",wolf_controller::_rpy[i].c_str(),_task_id.c_str(),nh_.getNamespace().c_str());
      use_identity = true;
    }
    // Check if the values are positive
    if(K(i,i)<0.0)
    {
      ROS_WARN("K gain must be positive!");
      use_identity = true;
    }
  }

  if(use_identity)
    K = Eigen::Matrix3d::Identity();

  buffer_kp_roll_   = K(0,0);
  buffer_kp_pitch_  = K(1,1);
  buffer_kp_yaw_    = K(2,2);

  setMomentumGain(K);
}

void AngularMomentum::updateCost(const Eigen::VectorXd& x)
{
  cost_ = computeCost(x);
}

void AngularMomentum::update(const Eigen::VectorXd& x)
{
  if(OPTIONS.set_ext_lambda)
    setLambda(buffer_lambda1_);
  if(OPTIONS.set_ext_weight)
    setWeight(buffer_weight_diag_);
  if(OPTIONS.set_ext_gains)
  {
    tmp_matrix3d_.setZero();
    tmp_matrix3d_(0,0) = buffer_kp_roll_;
    tmp_matrix3d_(1,1) = buffer_kp_pitch_;
    tmp_matrix3d_(2,2) = buffer_kp_yaw_;
    setMomentumGain(tmp_matrix3d_);
  }
  OpenSoT::tasks::acceleration::AngularMomentum::update(x);
}

void AngularMomentum::publish(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{

}
