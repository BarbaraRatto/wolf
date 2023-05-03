/**
 * @file com.cpp
 * @author Gennaro Raiola
 * @date 24 April, 2023
 * @brief This file contains the com task wrapper for ROS
 */

// WoLF
#include <wolf_controller/ros_wrappers/com.h>

Com::Com(ros::NodeHandle& nh, const XBot::ModelInterface& robot, const OpenSoT::AffineHelper& qddot)
  :OpenSoT::tasks::acceleration::CoM(robot,qddot)
  ,TaskRosWrapperInterface<wolf_msgs::ComTask>(_task_id,nh)
{
  // Create the reference subscriber
  reference_sub_ = nh.subscribe("reference/"+_task_id, 1000, &Com::referenceCallback, this);
}

void Com::registerReconfigurableVariables()
{
  double lambda1 = getLambda();
  double lambda2 = getLambda2();
  double weight  = getWeight()(0,0);
  ddr_server_->registerVariable<double>("set_lambda_1",    lambda1,     boost::bind(&TaskRosWrapperInterface::setLambda1,this,_1)    ,"set lambda 1"   ,0.0,1000.0);
  ddr_server_->registerVariable<double>("set_lambda_2",    lambda2,     boost::bind(&TaskRosWrapperInterface::setLambda2,this,_1)    ,"set lambda 2"   ,0.0,1000.0);
  ddr_server_->registerVariable<double>("set_weight_diag", weight,      boost::bind(&TaskRosWrapperInterface::setWeightDiag,this,_1) ,"set weight diag",0.0,1000.0);
  Eigen::Matrix3d Kp = getKp();
  ddr_server_->registerVariable<double>("kp_x",            Kp(0,0), boost::bind(&TaskRosWrapperInterface::setKpX,this,_1)            ,"Kp(0,0)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kp_y",            Kp(1,1), boost::bind(&TaskRosWrapperInterface::setKpY,this,_1)            ,"Kp(1,1)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kp_z",            Kp(2,2), boost::bind(&TaskRosWrapperInterface::setKpZ,this,_1)            ,"Kp(2,2)", 0.0, 10000.0);
  Eigen::Matrix3d Kd = getKd();
  ddr_server_->registerVariable<double>("kd_x",            Kd(0,0), boost::bind(&TaskRosWrapperInterface::setKdX,this,_1)            ,"Kd(0,0)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kd_y",            Kd(1,1), boost::bind(&TaskRosWrapperInterface::setKdY,this,_1)            ,"Kd(1,1)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kd_z",            Kd(2,2), boost::bind(&TaskRosWrapperInterface::setKdZ,this,_1)            ,"Kd(2,2)", 0.0, 10000.0);
  ddr_server_->publishServicesTopics();
}

void Com::loadParams()
{

  double lambda1, lambda2, weight;
  if (!nh_.getParam("gains/"+_task_id+"/lambda1" , lambda1))
  {
    ROS_WARN("No lambda1 gain given for task %s in the namespace: %s, using the default value loaded from the task",_task_id.c_str(),nh_.getNamespace().c_str());
    lambda1 = getLambda();
  }
  if (!nh_.getParam("gains/"+_task_id+"/lambda2" , lambda2))
  {
    ROS_WARN("No lambda2 gain given for task %s in the namespace: %s, using the default value loaded from the task",_task_id.c_str(),nh_.getNamespace().c_str());
    lambda2 = getLambda2();
  }
  if (!nh_.getParam("gains/"+_task_id+"/weight" , weight))
  {
    ROS_WARN("No weight gain given for task %s in the namespace: %s, using the default value loaded from the task",_task_id.c_str(),nh_.getNamespace().c_str());
    weight = getWeight()(0,0);
  }
  // Check if the values are positive
  if(lambda1 < 0 || lambda2 < 0 || weight < 0)
    throw std::runtime_error("Lambda and weight must be positive!");

  buffer_lambda1_ = lambda1;
  buffer_lambda2_ = lambda2;
  buffer_weight_diag_ = weight;

  setLambda(lambda1,lambda2);
  setWeight(weight);

  Eigen::Matrix3d Kp = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d Kd = Eigen::Matrix3d::Zero();
  bool use_identity = false;
  for(unsigned int i=0; i<wolf_controller::_xyz.size(); i++)
  {
    if (!nh_.getParam("gains/"+_task_id+"/Kp/" + wolf_controller::_xyz[i] , Kp(i,i)))
    {
      ROS_WARN("No Kp.%s gain given for task %s in the namespace: %s, using an identity matrix. ",wolf_controller::_xyz[i].c_str(),_task_id.c_str(),nh_.getNamespace().c_str());
      use_identity = true;
    }
    if (!nh_.getParam("gains/"+_task_id+"/Kd/"  + wolf_controller::_xyz[i] , Kd(i,i)))
    {
      ROS_WARN("No Kd.%s gain given for task %s in the namespace: %s, using an identity matrix. ",wolf_controller::_xyz[i].c_str(),_task_id.c_str(),nh_.getNamespace().c_str());
      use_identity = true;
    }
    // Check if the values are positive
    if(Kp(i,i)<0.0 || Kd(i,i)<0.0)
      throw std::runtime_error("Kp and Kd must be positive definite!");

  }

  if(use_identity)
  {
    Kp = Eigen::Matrix3d::Identity();
    Kd = Eigen::Matrix3d::Identity();
  }

  buffer_kp_x_     = Kp(0,0);
  buffer_kp_y_     = Kp(1,1);
  buffer_kp_z_     = Kp(2,2);

  buffer_kd_x_     = Kd(0,0);
  buffer_kd_y_     = Kd(1,1);
  buffer_kd_z_     = Kd(2,2);

  setKp(Kp);
  setKd(Kd);
}

void Com::updateCost(const Eigen::VectorXd& x)
{
  cost_ = computeCost(x);
}

void Com::publish(const ros::Time& time, const ros::Duration& /*period*/)
{
  if(rt_pub_->trylock())
  {
    rt_pub_->msg_.header.frame_id = getBaseLink();
    rt_pub_->msg_.header.stamp = time;

    // ACTUAL VALUES
    getActualPose(tmp_vector3d_);
    // Pose - Translation
    wolf_controller_utils::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.position_actual);
    // Velocity reference
    tmp_vector3d_ = getCachedVelocityReference();
    wolf_controller_utils::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.velocity_reference);

    // REFERENCE VALUES
    getReference(tmp_vector3d_);
    // Pose - Translation
    wolf_controller_utils::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.position_reference);

    // COST
    rt_pub_->msg_.cost = cost_;

    rt_pub_->unlockAndPublish();
  }
}

bool Com::reset()
{
  bool res = OpenSoT::tasks::acceleration::CoM::reset();
  getActualPose(tmp_vector3d_);
  buffer_reference_pos_.initRT(tmp_vector3d_);
  return res;
}

void Com::_update(const Eigen::VectorXd& x)
{
  if(OPTIONS.set_ext_lambda)
    setLambda(buffer_lambda1_,buffer_lambda2_);
  if(OPTIONS.set_ext_weight)
    setWeight(buffer_weight_diag_);
  if(OPTIONS.set_ext_gains)
  {
    tmp_matrix3d_.setZero();
    tmp_matrix3d_(0,0) = buffer_kp_x_;
    tmp_matrix3d_(1,1) = buffer_kp_y_;
    tmp_matrix3d_(2,2) = buffer_kp_z_;
    setKp(tmp_matrix3d_);
    tmp_matrix3d_.setZero();
    tmp_matrix3d_(0,0) = buffer_kd_x_;
    tmp_matrix3d_(1,1) = buffer_kd_y_;
    tmp_matrix3d_(2,2) = buffer_kd_z_;
    setKd(tmp_matrix3d_);
  }
  if(OPTIONS.set_ext_reference)
  {
    // Set external reference
    setReference(*buffer_reference_pos_.readFromRT(),*buffer_reference_vel_.readFromRT());
  }
  OpenSoT::tasks::acceleration::CoM::_update(x);
}

void Com::referenceCallback(const wolf_msgs::Com::ConstPtr& msg)
{
  double period = wolf_controller::_period;

  if(last_time_ != 0.0)
    period = msg->header.stamp.toSec() - last_time_;

  Eigen::Vector3d position_reference = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_reference = Eigen::Vector3d::Zero();

  position_reference.x() = msg->position.x;
  position_reference.y() = msg->position.y;
  position_reference.z() = msg->position.z;

  velocity_reference.x() = msg->velocity.x;
  velocity_reference.y() = msg->velocity.y;
  velocity_reference.z() = msg->velocity.z;

  buffer_reference_pos_.writeFromNonRT(position_reference);
  buffer_reference_vel_.writeFromNonRT(velocity_reference);

  last_time_ = msg->header.stamp.toSec();
}
