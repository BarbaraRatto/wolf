/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef ROS_WRAPPERS_COM_H
#define ROS_WRAPPERS_COM_H

// OpenSoT
#include <OpenSoT/Task.h>
#include <OpenSoT/tasks/acceleration/CoM.h>

// ROS
#include <wolf_msgs/ComTask.h>

// WoLF
#include <wolf_controller/ros_wrappers/interface.h>

// WoLF utils
#include <wolf_controller_utils/converters.h>

// CoM
class CoM : public OpenSoT::tasks::acceleration::CoM, public TaskRosWrapperInterface<wolf_msgs::ComTask>
{

public:

  typedef std::shared_ptr<CoM> Ptr;

  CoM(ros::NodeHandle& nh, const XBot::ModelInterface& robot, const OpenSoT::AffineHelper& qddot)
    :OpenSoT::tasks::acceleration::CoM(robot,qddot)
    ,TaskRosWrapperInterface<wolf_msgs::ComTask>(_task_id,nh)
  {
    // Setup the interpolator
    //trj_ = std::make_shared<wolf_controller::CartesianTrajectory>(this);

    // Create the reference subscriber
    reference_sub_ = nh.subscribe("reference/"+_task_id, 1000, &CoM::referenceCallback, this);
  }

  virtual void registerReconfigurableVariables() override
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

  virtual void loadParams() override
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

  virtual void updateCost(const Eigen::VectorXd& x) override
  {
    cost_ = computeCost(x);
  }

  virtual void publish(const ros::Time& time) override
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

  virtual void _update(const Eigen::VectorXd& x) override
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
        // Interpolation
        //trj_->update(wolf_controller::_period);
        //trj_->getReference(tmp_affine3d_,&tmp_vector6d_,&tmp_vector6d_1_);
        //setReference(tmp_affine3d_.translation(),tmp_vector6d_.head(3),tmp_vector6d_1_.head(3));
    }
    OpenSoT::tasks::acceleration::CoM::_update(x);
  }

private:

  void referenceCallback(const wolf_msgs::ComTask::ConstPtr& msg)
  {
      double period = wolf_controller::_period;

      if(last_time_ != 0.0)
        period = msg->header.stamp.toSec() - last_time_;

      Eigen::Affine3d pose_reference = Eigen::Affine3d::Identity();
      pose_reference.translation().x() = msg->position_reference.x;
      pose_reference.translation().y() = msg->position_reference.y;
      pose_reference.translation().z() = msg->position_reference.z;
      //trj_->setWayPoint(pose_reference,period);

      last_time_ = msg->header.stamp.toSec();
  }

};

#endif // ROS_WRAPPERS_COM_H

