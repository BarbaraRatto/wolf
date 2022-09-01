/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef ROS_WRAPPERS_POSTURAL_H
#define ROS_WRAPPERS_POSTURAL_H

// OpenSoT
#include <OpenSoT/Task.h>
#include <OpenSoT/tasks/acceleration/Postural.h>

// ROS
#include <wolf_msgs/PosturalTask.h>

// WoLF
#include <wolf_controller/ros_wrappers/interface.h>
#include <wolf_controller/geometry.h>
#include <wolf_controller/utils.h>

// POSTURAL
class Postural : public OpenSoT::tasks::acceleration::Postural, public TaskRosWrapperInterface<wolf_msgs::PosturalTask>
{

public:

  typedef std::shared_ptr<Postural> Ptr;

  Postural(ros::NodeHandle& nh, const XBot::ModelInterface& robot,
           OpenSoT::AffineHelper qddot = OpenSoT::AffineHelper(), const std::string task_id = "Postural")
    :OpenSoT::tasks::acceleration::Postural(robot,qddot,task_id)
    ,TaskRosWrapperInterface<wolf_msgs::PosturalTask>(task_id,nh)
  {
    const unsigned int& size = getActualPositions().size();
    tmp_vectorXd_.resize(size);
    rt_pub_->msg_.name.resize(size);
    rt_pub_->msg_.position_actual.resize(size);
    rt_pub_->msg_.velocity_actual.resize(size);
    rt_pub_->msg_.position_reference.resize(size);
    rt_pub_->msg_.velocity_reference.resize(size);
    rt_pub_->msg_.position_error.resize(size);
    rt_pub_->msg_.velocity_error.resize(size);
  }

  virtual void registerReconfigurableVariables() override
  {
    double lambda1 = getLambda();
    double lambda2 = getLambda2();
    double weight  = getWeight()(0,0);
    ddr_server_->registerVariable<double>("set_lambda_1",    lambda1,     boost::bind(&TaskRosWrapperInterface::setLambda1,this,_1)    ,"set lambda 1"   ,0.0,1000.0);
    ddr_server_->registerVariable<double>("set_lambda_2",    lambda2,     boost::bind(&TaskRosWrapperInterface::setLambda2,this,_1)    ,"set lambda 2"   ,0.0,1000.0);
    ddr_server_->registerVariable<double>("set_weight_diag", weight,      boost::bind(&TaskRosWrapperInterface::setWeightDiag,this,_1) ,"set weight diag",0.0,1000.0);
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
  }

  virtual void updateCost(const Eigen::VectorXd& x) override
  {
    cost_ = computeCost(x);
  }

  virtual void publish(const ros::Time& time)
  {
    if(rt_pub_->trylock())
    {
      rt_pub_->msg_.header.frame_id = "Joints";
      rt_pub_->msg_.header.stamp = time;

      for(unsigned int i = 0;i<getActualPositions().size();i++)
      {
        rt_pub_->msg_.name[i] = wolf_controller::_dof_names[i];
        rt_pub_->msg_.position_actual[i] = getActualPositions()(i);
        rt_pub_->msg_.position_reference[i] = getReference()(i);
        rt_pub_->msg_.velocity_actual[i] =  0.0;
        rt_pub_->msg_.velocity_reference[i] = getCachedVelocityReference()(i);
        rt_pub_->msg_.position_error[i] = getError()(i);
        rt_pub_->msg_.velocity_error[i] = getVelocityError()(i);
      }

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
    OpenSoT::tasks::acceleration::Postural::_update(x);
  }
};

#endif // ROS_WRAPPERS_POSTURAL_H

