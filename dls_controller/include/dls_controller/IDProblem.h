#ifndef _IDPROBLEM_H_
#define _IDPROBLEM_H_

// OpenSoT
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <OpenSoT/tasks/acceleration/CoM.h>
#include <OpenSoT/constraints/acceleration/DynamicFeasibility.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/utils/InverseDynamics.h>
#include <OpenSoT/constraints/force/FrictionCone.h>
#include <OpenSoT/constraints/force/WrenchLimits.h>

// ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <realtime_tools/realtime_publisher.h>
#include <dls_controller/CartesianTask.h>
#include <dls_controller/JointsTask.h>
#include <dls_controller/TaskConfig.h>

namespace OpenSoT{

class TaskRosWrapperInterface
{

public:

   typedef std::shared_ptr<TaskRosWrapperInterface> Ptr;

   virtual ~TaskRosWrapperInterface(){}

   virtual void publish(const ros::Time& /*time*/) = 0;
   virtual void dynamicReconfigureCallback(dls_controller::TaskConfig& /*config*/, uint32_t /*level*/) = 0;
   virtual void dynamicReconfigureUpdate() = 0;

protected:

    std::shared_ptr<dynamic_reconfigure::Server<dls_controller::TaskConfig>> server_;
    dls_controller::TaskConfig default_config_;

    Eigen::Affine3d       tmp_affine3d_;
    Eigen::VectorXd       tmp_vectorxd_;
    Eigen::Vector3d       tmp_vector3d_;
    Eigen::Vector6d       tmp_vector6d_;
    Eigen::Quaterniond    tmp_quaterniond_;

};

template <typename task_ptr_t, typename msg_t>
class TaskRosWrapperBase : public TaskRosWrapperInterface
{

public:

   TaskRosWrapperBase(ros::NodeHandle& nh, task_ptr_t task)
   {
        assert(task);
        task_ = task;

        rt_pub_.reset(new realtime_tools::RealtimePublisher<msg_t>(nh, task->getTaskID(), 4));

        //ros::NodeHandle task_nh(nh.getNamespace()+"/"+task->getTaskID());
        ros::NodeHandle task_nh(task->getTaskID());
        server_.reset(new dynamic_reconfigure::Server<dls_controller::TaskConfig>(task_nh));
        server_->setCallback(boost::bind(&TaskRosWrapperBase::dynamicReconfigureCallback, this, _1, _2));
   }

   void dynamicReconfigureCallback(dls_controller::TaskConfig &config, uint32_t level)
   {
       switch(level)
       {
       case 0:
           task_->setLambda(config.lambda1,config.lambda2);
           break;
       }
   }

   void dynamicReconfigureUpdate()
   {
        default_config_.lambda1 = task_->getLambda();
        default_config_.lambda2 = 0.0;//task_->getLambda(); FIXME
        if(server_) server_->updateConfig(default_config_);
   }

   virtual void publish(const ros::Time& /*time*/) = 0;

protected:

    std::shared_ptr<realtime_tools::RealtimePublisher<msg_t>> rt_pub_;
    task_ptr_t task_;
};

template <typename task_ptr_t, typename msg_t>
class TaskRosWrapper : public TaskRosWrapperBase<task_ptr_t,msg_t>
{
public:
    TaskRosWrapper(ros::NodeHandle& nh, task_ptr_t task)
    {
        TaskRosWrapperBase<task_ptr_t,msg_t>(nh,task);
    }
};

// Specializations
template <>
class TaskRosWrapper<tasks::acceleration::Cartesian::Ptr,dls_controller::CartesianTask> : public TaskRosWrapperBase<tasks::acceleration::Cartesian::Ptr,dls_controller::CartesianTask>
{

public:

    TaskRosWrapper(ros::NodeHandle& nh, tasks::acceleration::Cartesian::Ptr task):
        TaskRosWrapperBase<tasks::acceleration::Cartesian::Ptr,dls_controller::CartesianTask>(nh,task){}

    virtual void publish(const ros::Time& time)
    {
        if(rt_pub_->trylock())
        {
            rt_pub_->msg_.header.frame_id = task_->getBaseLink();
            rt_pub_->msg_.header.stamp = time;

            // ACTUAL VALUES
            task_->getActualPose(tmp_affine3d_);
            task_->getActualTwist(tmp_vector6d_);
            // Transform the R matrix into a quaternion
            tmp_quaterniond_ = tmp_affine3d_.linear();
            // Pose - Translation
            rt_pub_->msg_.pose_actual.position.x = tmp_affine3d_.translation().x();
            rt_pub_->msg_.pose_actual.position.y = tmp_affine3d_.translation().y();
            rt_pub_->msg_.pose_actual.position.z = tmp_affine3d_.translation().z();
            // Pose - Linear
            rt_pub_->msg_.pose_actual.orientation.x = tmp_quaterniond_.x();
            rt_pub_->msg_.pose_actual.orientation.y = tmp_quaterniond_.y();
            rt_pub_->msg_.pose_actual.orientation.z = tmp_quaterniond_.z();
            rt_pub_->msg_.pose_actual.orientation.w = tmp_quaterniond_.w();
            // Twist
            rt_pub_->msg_.twist_actual.linear.x  = tmp_vector6d_(0);
            rt_pub_->msg_.twist_actual.linear.y  = tmp_vector6d_(1);
            rt_pub_->msg_.twist_actual.linear.z  = tmp_vector6d_(2);
            rt_pub_->msg_.twist_actual.angular.x = tmp_vector6d_(3);
            rt_pub_->msg_.twist_actual.angular.y = tmp_vector6d_(4);
            rt_pub_->msg_.twist_actual.angular.z = tmp_vector6d_(5);

            // REFERENCE VALUES
            task_->getReference(tmp_affine3d_,tmp_vector6d_);
            // Transform the R matrix into a quaternion
            tmp_quaterniond_ = tmp_affine3d_.linear();
            // Pose - Translation
            rt_pub_->msg_.pose_reference.position.x = tmp_affine3d_.translation().x();
            rt_pub_->msg_.pose_reference.position.y = tmp_affine3d_.translation().y();
            rt_pub_->msg_.pose_reference.position.z = tmp_affine3d_.translation().z();
            // Pose - Linear
            rt_pub_->msg_.pose_reference.orientation.x = tmp_quaterniond_.x();
            rt_pub_->msg_.pose_reference.orientation.y = tmp_quaterniond_.y();
            rt_pub_->msg_.pose_reference.orientation.z = tmp_quaterniond_.z();
            rt_pub_->msg_.pose_reference.orientation.w = tmp_quaterniond_.w();
            // Twist
            rt_pub_->msg_.twist_reference.linear.x  = tmp_vector6d_(0);
            rt_pub_->msg_.twist_reference.linear.y  = tmp_vector6d_(1);
            rt_pub_->msg_.twist_reference.linear.z  = tmp_vector6d_(2);
            rt_pub_->msg_.twist_reference.angular.x = tmp_vector6d_(3);
            rt_pub_->msg_.twist_reference.angular.y = tmp_vector6d_(4);
            rt_pub_->msg_.twist_reference.angular.z = tmp_vector6d_(5);

            rt_pub_->unlockAndPublish();
        }
    }
};


template <>
class TaskRosWrapper<tasks::acceleration::CoM::Ptr,dls_controller::CartesianTask> : public TaskRosWrapperBase<tasks::acceleration::CoM::Ptr,dls_controller::CartesianTask>
{

public:

    TaskRosWrapper(ros::NodeHandle& nh, tasks::acceleration::CoM::Ptr task):
        TaskRosWrapperBase<tasks::acceleration::CoM::Ptr,dls_controller::CartesianTask>(nh,task){}

    virtual void publish(const ros::Time& time)
    {
        if(rt_pub_->trylock())
        {
            rt_pub_->msg_.header.frame_id = task_->getBaseLink();
            rt_pub_->msg_.header.stamp = time;

            // ACTUAL VALUES
            task_->getActualPose(tmp_vector3d_);
            // Pose - Translation
            rt_pub_->msg_.pose_actual.position.x = tmp_vector3d_(0);
            rt_pub_->msg_.pose_actual.position.y = tmp_vector3d_(1);
            rt_pub_->msg_.pose_actual.position.z = tmp_vector3d_(2);

            // REFERENCE VALUES
            task_->getReference(tmp_vector3d_);
            // Pose - Translation
            rt_pub_->msg_.pose_reference.position.x = tmp_vector3d_(0);
            rt_pub_->msg_.pose_reference.position.y = tmp_vector3d_(1);
            rt_pub_->msg_.pose_reference.position.z = tmp_vector3d_(2);

            rt_pub_->unlockAndPublish();
        }
    }

};

template <>
class TaskRosWrapper<tasks::acceleration::Postural::Ptr,dls_controller::JointsTask> : public TaskRosWrapperBase<tasks::acceleration::Postural::Ptr,dls_controller::JointsTask>
{

public:

    TaskRosWrapper(ros::NodeHandle& nh, tasks::acceleration::Postural::Ptr task):
        TaskRosWrapperBase<tasks::acceleration::Postural::Ptr,dls_controller::JointsTask>(nh,task)
    {
        const unsigned int& size = task_->getActualPositions().size();
        tmp_vectorxd_.resize(size);
        rt_pub_->msg_.position_actual.resize(size);
        rt_pub_->msg_.velocity_actual.resize(size);
        rt_pub_->msg_.position_reference.resize(size);
        rt_pub_->msg_.velocity_reference.resize(size);
        rt_pub_->msg_.position_error.resize(size);
        rt_pub_->msg_.velocity_error.resize(size);
    }

    virtual void publish(const ros::Time& time)
    {
        if(rt_pub_->trylock())
        {
            rt_pub_->msg_.header.frame_id = "Joints";
            rt_pub_->msg_.header.stamp = time;

             // FIXME Is it RT safe with the move operator?
            auto position_actual = task_->getActualPositions();
            auto position_reference = task_->getReference();
            auto velocity_error = task_->getVelocityError();
            auto position_error = task_->getError();

            for(unsigned int i = 0;i<tmp_vectorxd_.size();i++)
            {
                rt_pub_->msg_.position_actual[i] = position_actual(i);
                rt_pub_->msg_.position_reference[i] = position_reference(i);
                rt_pub_->msg_.velocity_actual[i] = 0.0;
                rt_pub_->msg_.velocity_reference[i] = 0.0;
                rt_pub_->msg_.position_error[i] = position_error(i);
                rt_pub_->msg_.velocity_error[i] = velocity_error(i);
            }


            rt_pub_->unlockAndPublish();

        }
    }

};


/**
 * @brief The IDProblem class wraps the tasks, constraints and the solver used to solve the ID
 */
class IDProblem{

public:
    typedef boost::shared_ptr<IDProblem> Ptr;
    typedef TaskRosWrapper<tasks::acceleration::Cartesian::Ptr,dls_controller::CartesianTask> CartesianWrapper;
    typedef TaskRosWrapper<tasks::acceleration::CoM::Ptr,dls_controller::CartesianTask> ComWrapper;
    typedef TaskRosWrapper<tasks::acceleration::Postural::Ptr,dls_controller::JointsTask> PosturalWrapper;

    /**
     * @brief IDProblem constructor
     * @param ros node handle
     * @param model pointer to external model
     * @param dT control loop
     * @param vector of contact links name
     */
    IDProblem(ros::NodeHandle& nh, XBot::ModelInterface::Ptr model, const double dT, std::vector<std::string>& contact_links);
    ~IDProblem();

    /**
     * @brief solve call this after update()
     * @param x solution form solver (tau)
     * @return true if solved
     */
    bool solve(Eigen::VectorXd& x);

    /**
     * @brief get the ground reaction forces call this after solve()
     * @param grfs from the solver
     */
    void getGroundReactionForces(Eigen::VectorXd& grfs);

    /**
     * @brief update call after the model.update() to update the autostack
     * @param x input state q
     */
    void update();

    /**
     * @brief log to log solver and autostaack status
     * @param logger a pointer to a MatLogger
     */
    void log(XBot::MatLogger::Ptr& logger);

    /**
     * @brief publish the ros topics related to the tasks
     * @param ros current time
     */
    void publish(const ros::Time& time);

    /**
     * @brief Cartesian tasks
     */
    std::map<std::string,tasks::acceleration::Cartesian::Ptr> _feet;
    tasks::acceleration::Cartesian::Ptr _waist;
    tasks::acceleration::CoM::Ptr _com;

    /**
     * @brief Expose the tasks to ROS
     */
    std::vector<TaskRosWrapperInterface::Ptr> _tasks_ros;

    /**
     * @brief _posture a postural task
     */
    tasks::acceleration::Postural::Ptr _postural;

    /**
     * @brief _x_lims some bounds
     */
    constraints::GenericConstraint::Ptr _qddot_lims;

    /**
     * @brief wrench bounds
     */
    constraints::force::WrenchesLimits::Ptr _wrenches_lims;

    /**
     * @brief _model
     */
    XBot::ModelInterface::Ptr _model;

private:
    /**
     * @brief _dynamics constraint relates the floating base with the contact forces
     */
    constraints::acceleration::DynamicFeasibility::Ptr _dynamics;

    /**
     * @brief _friction_cones constraints
     */
    constraints::force::FrictionCones::Ptr _friction_cones;

    /**
     * @brief _id_problem the final ID problem
     */
    AutoStack::Ptr _id_problem;

    /**
     * @brief _solver iHQP solver
     */
    solvers::iHQP::Ptr _solver;

    /**
     * @brief _id inverse dynamics computation & variable helper
     */
    OpenSoT::utils::InverseDynamics::Ptr _id;

    /**
     * @brief _x decision variables
     */
    Eigen::VectorXd _x;

    Eigen::VectorXd _qddot;

};



}

#endif

