#ifndef ID_PROBLEM_H
#define ID_PROBLEM_H

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
#include <interactive_markers/interactive_marker_server.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <wb_controller/CartesianTask.h>
#include <wb_controller/JointsTask.h>
#include <wb_controller/taskConfig.h>
#include <wb_controller/problemConfig.h>

// WB
#include <wb_controller/geometry.h>
#include <wb_controller/utils.h>
#include <wb_controller/Gains.h>

// STD
#include <atomic>

namespace OpenSoT{

class TaskRosWrapperInterface
{

public:

    typedef std::shared_ptr<TaskRosWrapperInterface> Ptr;

    TaskRosWrapperInterface(){spinner_.reset(new ros::AsyncSpinner(1)); spinner_->start();}
    virtual ~TaskRosWrapperInterface(){spinner_->stop();}

    virtual void publish(const ros::Time& /*time*/) = 0;
    virtual void dynamicReconfigureCallback(wb_controller::taskConfig& /*config*/, uint32_t /*level*/) = 0;
    virtual void dynamicReconfigureUpdate() = 0;
    virtual void updateReference() {}

protected:

    std::shared_ptr<dynamic_reconfigure::Server<wb_controller::taskConfig>> server_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_;
    wb_controller::taskConfig default_config_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;
    ros::ServiceServer service_;

    Eigen::Affine3d       tmp_affine3d_;
    Eigen::VectorXd       tmp_vectorxd_;
    Eigen::Vector3d       tmp_vector3d_;
    Eigen::Vector6d       tmp_vector6d_;
    Eigen::Quaterniond    tmp_quaterniond_;

    realtime_tools::RealtimeBuffer<Eigen::Affine3d> rt_affine3d_;

};

template <typename task_ptr_t, typename msg_t>
class TaskRosWrapperBase : public TaskRosWrapperInterface
{

public:

    typedef std::pair<double,double> gains_t;
    typedef std::map<std::string,gains_t> gains_map_t;

    TaskRosWrapperBase(ros::NodeHandle& nh, task_ptr_t task)
    {
        assert(task);
        task_ = task;

        rt_pub_.reset(new realtime_tools::RealtimePublisher<msg_t>(nh, task->getTaskID(), 4));

        //ros::NodeHandle task_nh(nh.getNamespace()+"/"+task->getTaskID());
        ros::NodeHandle task_nh(task->getTaskID());
        server_.reset(new dynamic_reconfigure::Server<wb_controller::taskConfig>(task_nh));
        server_->setCallback(boost::bind(&TaskRosWrapperBase::dynamicReconfigureCallback, this, _1, _2));

        service_ = nh.advertiseService(task->getTaskID()+"/set_gains",&TaskRosWrapperBase::setGains,this);
    }

    void dynamicReconfigureCallback(wb_controller::taskConfig &config, uint32_t level)
    {
        switch(level)
        {
        case 0:
            task_->setLambda(config.lambda1,config.lambda2);
            break;
        case 1:
            if(config.weight_diag>=0)
            {
                Eigen::MatrixXd new_weight = Eigen::MatrixXd::Identity(task_->getTaskSize(),task_->getTaskSize()) * config.weight_diag;
                Eigen::MatrixXd old_weight = Eigen::MatrixXd::Zero(task_->getTaskSize(),task_->getTaskSize());
                old_weight = task_->getWeight();
                task_->setWeight(new_weight);
                ROS_INFO_STREAM("Set weight diagonal values for task "<<task_->getTaskID()<<" from "<< old_weight(0,0) <<" to "<<new_weight(0,0));
            }
            else
                ROS_WARN("Weight diagonal value has to be positive definite!");
            break;
        }
    }

    void dynamicReconfigureUpdate()
    {
        default_config_.lambda1 = task_->getLambda();
        default_config_.lambda2 = task_->getLambda2();
        default_config_.weight_diag = task_->getWeight()(0,0); // Note: we take the first value of the diagonal.
        if(server_) server_->updateConfig(default_config_);
    }

    virtual bool setGains(wb_controller::Gains::Request  &req, wb_controller::Gains::Response &res)
    {
        ROS_WARN("setGains not implemented yet.");
        return false;
    }

    virtual void publish(const ros::Time& /*time*/) = 0;

protected:

    std::shared_ptr<realtime_tools::RealtimePublisher<msg_t>> rt_pub_;
    task_ptr_t task_;
    gains_map_t gains_map_;

    Eigen::MatrixXd Kp_;
    Eigen::MatrixXd Kd_;
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
class TaskRosWrapper<tasks::acceleration::Cartesian::Ptr,wb_controller::CartesianTask> : public TaskRosWrapperBase<tasks::acceleration::Cartesian::Ptr,wb_controller::CartesianTask>
{

public:

    TaskRosWrapper(ros::NodeHandle& nh, tasks::acceleration::Cartesian::Ptr task):
        TaskRosWrapperBase<tasks::acceleration::Cartesian::Ptr,wb_controller::CartesianTask>(nh,task)
    {

        task_->getActualPose(tmp_affine3d_);

        rt_affine3d_.initRT(tmp_affine3d_);

        tmp_quaterniond_ = tmp_affine3d_.linear();

        marker_.reset(new interactive_markers::InteractiveMarkerServer(task_->getTaskID()));

        // create an interactive marker for our server
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = task_->getBaseLink();
        int_marker.name = task_->getTaskID();
        int_marker.description = task_->getTaskID();

        wb_controller::affine3dToPose(tmp_affine3d_,int_marker.pose);

        // create a grey box marker
        visualization_msgs::Marker box_marker;
        box_marker.type = visualization_msgs::Marker::SPHERE;
        box_marker.scale.x = 0.2;
        box_marker.scale.y = 0.2;
        box_marker.scale.z = 0.2;
        box_marker.color.r = 0.5;
        box_marker.color.g = 0.5;
        box_marker.color.b = 0.5;
        box_marker.color.a = 0.5;

        // create a non-interactive control which contains the box
        visualization_msgs::InteractiveMarkerControl box_control;
        box_control.always_visible = true;
        box_control.markers.push_back( box_marker );

        // add the control to the interactive marker
        int_marker.controls.push_back( box_control );

        visualization_msgs::InteractiveMarkerControl control;

        control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        // add the interactive marker to our collection &
        // tell the server to call processFeedback() when feedback arrives for it
        marker_->insert(int_marker, boost::bind(&TaskRosWrapper::processFeedback, this, _1));

        // 'commit' changes and send to all clients
        marker_->applyChanges();

        Kp_ = task_->getKp();
        Kd_ = task_->getKd();
        // NOTE: by default we use this order: x y z roll pitch yaw
        for(unsigned int i=0; i<wb_controller::_cartesian_names.size(); i++)
        {
            gains_map_[wb_controller::_dof_names[i]] = gains_t(Kp_(i,i),Kd_(i,i));
        }

        //Kp_ = Eigen::MatrixXd::Zero(6,6);
        //Kd_ = Eigen::MatrixXd::Zero(6,6);
    }

    virtual void publish(const ros::Time& time)
    {
        if(rt_pub_->trylock())
        {
            rt_pub_->msg_.header.frame_id = task_->getBaseLink();
            rt_pub_->msg_.header.stamp = time;

            // ACTUAL VALUES
            task_->getActualPose(tmp_affine3d_);
            task_->getActualTwist(tmp_vector6d_);
            wb_controller::affine3dToPose(tmp_affine3d_,rt_pub_->msg_.pose_actual);
            wb_controller::vector6dToTwist(tmp_vector6d_,rt_pub_->msg_.twist_actual);

            // REFERENCE VALUES
            task_->getReference(tmp_affine3d_);
            tmp_vector6d_ = task_->getCachedVelocityReference();
            wb_controller::affine3dToPose(tmp_affine3d_,rt_pub_->msg_.pose_reference);
            wb_controller::vector6dToTwist(tmp_vector6d_,rt_pub_->msg_.twist_reference);

            rt_pub_->unlockAndPublish();
        }
    }

    virtual void updateReference()
    {
        // Set the task reference for the cartesian task
        task_->setReference(*rt_affine3d_.readFromRT());
    }

    virtual bool setGains(wb_controller::Gains::Request  &req, wb_controller::Gains::Response &res)
    {
        for(unsigned int i=0; i<req.name.size();i++)
        {
            if(gains_map_.find(req.name[i]) != gains_map_.end())
            {
                if(req.Kp[i]>=0 && req.Kd[i]>=0)
                {
                    gains_map_[req.name[i]] = gains_t(req.Kp[i],req.Kd[i]);
                }
                else
                {
                    ROS_WARN_STREAM("Variable: "<<req.name[i]<<" Kp and Kd have to be positive!");
                }
            }
            else
            {
                ROS_WARN_STREAM("Variable: "<<req.name[i] << " does not exist!");
            }
        }

        res.name.resize(gains_map_.size());
        res.Kp.resize(gains_map_.size());
        res.Kd.resize(gains_map_.size());
        for (unsigned int i = 0; i<wb_controller::_cartesian_names.size(); i++)
        {
            res.Kp[i] = Kp_(i,i) = gains_map_[wb_controller::_cartesian_names[i]].first;
            res.Kd[i] = Kd_(i,i) = gains_map_[wb_controller::_cartesian_names[i]].second;
            res.name[i] = wb_controller::_cartesian_names[i];
        }

        task_->setGains(Kp_,Kd_);

        return true;
    }


protected:

    virtual void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        ROS_INFO_STREAM( feedback->marker_name << " is now at "
                         << feedback->pose.position.x << ", " << feedback->pose.position.y
                         << ", " << feedback->pose.position.z );

        Eigen::Vector3d translation_reference(feedback->pose.position.x,feedback->pose.position.y,feedback->pose.position.z);
        Eigen::Quaterniond orientation_reference(feedback->pose.orientation.w,feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z);
        Eigen::Affine3d pose_reference = Eigen::Affine3d::Identity();
        Eigen::Matrix3d R;

        wb_controller::quatToRotMat(orientation_reference,R);

        pose_reference.translation() = translation_reference;
        pose_reference.linear() = R;

        /*quaternion.x() = feedback->pose.orientation.x;
        quaternion.y() = feedback->pose.orientation.y;
        quaternion.z() = feedback->pose.orientation.z;
        quaternion.w() = feedback->pose.orientation.w;
        quatToRotMat(quaternion,world_R_task);*/

        // FIXME no orientation yet
        rt_affine3d_.writeFromNonRT(pose_reference);
    }
};


template <>
class TaskRosWrapper<tasks::acceleration::CoM::Ptr,wb_controller::CartesianTask> : public TaskRosWrapperBase<tasks::acceleration::CoM::Ptr,wb_controller::CartesianTask>
{

public:

    TaskRosWrapper(ros::NodeHandle& nh, tasks::acceleration::CoM::Ptr task):
        TaskRosWrapperBase<tasks::acceleration::CoM::Ptr,wb_controller::CartesianTask>(nh,task){}

    virtual void publish(const ros::Time& time)
    {
        if(rt_pub_->trylock())
        {
            rt_pub_->msg_.header.frame_id = task_->getBaseLink();
            rt_pub_->msg_.header.stamp = time;

            // ACTUAL VALUES
            task_->getActualPose(tmp_vector3d_);
            // Pose - Translation
            wb_controller::vector3dToPosePosition(tmp_vector3d_,rt_pub_->msg_.pose_actual);

            // REFERENCE VALUES
            task_->getReference(tmp_vector3d_);
            // Pose - Translation
            wb_controller::vector3dToPosePosition(tmp_vector3d_,rt_pub_->msg_.pose_reference);

            rt_pub_->unlockAndPublish();
        }
    }

};

template <>
class TaskRosWrapper<tasks::acceleration::Postural::Ptr,wb_controller::JointsTask> : public TaskRosWrapperBase<tasks::acceleration::Postural::Ptr,wb_controller::JointsTask>
{

public:

    TaskRosWrapper(ros::NodeHandle& nh, tasks::acceleration::Postural::Ptr task):
        TaskRosWrapperBase<tasks::acceleration::Postural::Ptr,wb_controller::JointsTask>(nh,task)
    {
        const unsigned int& size = task_->getActualPositions().size();
        tmp_vectorxd_.resize(size);
        rt_pub_->msg_.name.resize(size);
        rt_pub_->msg_.position_actual.resize(size);
        rt_pub_->msg_.velocity_actual.resize(size);
        rt_pub_->msg_.position_reference.resize(size);
        rt_pub_->msg_.velocity_reference.resize(size);
        rt_pub_->msg_.position_error.resize(size);
        rt_pub_->msg_.velocity_error.resize(size);

        Kp_ = task_->getKp();
        Kd_ = task_->getKd();
        // NOTE: by default we use the same leg order as RBDL (alphabetic order)
        for(unsigned int i=0; i<wb_controller::_dof_names.size(); i++)
        {
            gains_map_[wb_controller::_dof_names[i]] = gains_t(Kp_(i,i),Kd_(i,i));
        }

        //Kp_ = Eigen::MatrixXd::Zero(size,size);
        //Kd_ = Eigen::MatrixXd::Zero(size,size);
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
                rt_pub_->msg_.name[i] = wb_controller::_dof_names[i];
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

    // FIXME this is copied and pasted...
    virtual bool setGains(wb_controller::Gains::Request  &req, wb_controller::Gains::Response &res)
    {
        for(unsigned int i=0; i<req.name.size();i++)
        {
            if(gains_map_.find(req.name[i]) != gains_map_.end())
            {
                if(req.Kp[i]>=0 && req.Kd[i]>=0)
                {
                    gains_map_[req.name[i]] = gains_t(req.Kp[i],req.Kd[i]);
                }
                else
                {
                    ROS_WARN_STREAM("Variable: "<<req.name[i]<<" Kp and Kd have to be positive!");
                }
            }
            else
            {
                ROS_WARN_STREAM("Variable: "<<req.name[i] << " does not exist!");
            }
        }

        res.name.resize(gains_map_.size());
        res.Kp.resize(gains_map_.size());
        res.Kd.resize(gains_map_.size());

        for (unsigned int i = 0; i<wb_controller::_dof_names.size(); i++)
        {
            res.Kp[i] = Kp_(i,i) = gains_map_[wb_controller::_dof_names[i]].first;
            res.Kd[i] = Kd_(i,i) = gains_map_[wb_controller::_dof_names[i]].second;
            res.name[i] = wb_controller::_dof_names[i];
        }

        task_->setGains(Kp_,Kd_);


        return true;
    }

};


/**
 * @brief The IDProblem class wraps the tasks, constraints and the solver used to solve the ID
 */
class IDProblem
{

public:
    typedef boost::shared_ptr<IDProblem> Ptr;
    typedef TaskRosWrapper<tasks::acceleration::Cartesian::Ptr,wb_controller::CartesianTask> CartesianWrapper;
    typedef TaskRosWrapper<tasks::acceleration::CoM::Ptr,wb_controller::CartesianTask> ComWrapper;
    typedef TaskRosWrapper<tasks::acceleration::Postural::Ptr,wb_controller::JointsTask> PosturalWrapper;

    /**
     * @brief IDProblem constructor
     * @param ros node handle
     * @param model pointer to external model
     * @param vector of contact links name
     */
    IDProblem(ros::NodeHandle& nh, XBot::ModelInterface::Ptr model, std::vector<std::string> feet_names, std::string arm_tip_name = std::string());
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
     * @brief reset the tasks
     */
    void reset();

    /**
     * @brief update the tasks with the new external references (i.e. the references coming from the interactive markers)
     * @param task name to update
     */
    void updateReference(const std::string& task_name);

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
     * @brief set the mu parameter for the friction cones
     * @param mu value
     */
    void setFrictionConesMu(const double& mu);

    /**
     * @brief set the lower bound for the wrench limits along the selected axis (w.r.t world)
     * @param lower bound values
     */
    void setLowerForceBound(const double& x_force,const double& y_force,const double& z_force);


    /**
         * @brief Ros dynamic reconfigure callback
         */
    void dynamicReconfigureCallback(wb_controller::problemConfig &config, uint32_t level);

    /**
     * @brief Cartesian tasks
     */
    std::map<std::string,tasks::acceleration::Cartesian::Ptr> _feet;
    tasks::acceleration::Cartesian::Ptr _arm;
    tasks::acceleration::Cartesian::Ptr _waistRPY;
    tasks::acceleration::Cartesian::Ptr _waistZ;
    tasks::acceleration::CoM::Ptr _com;

    /**
     * @brief Expose the tasks to ROS
     */
    std::map<std::string,TaskRosWrapperInterface::Ptr> _tasks_ros;

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
         * @brief Update the dynamic reconfigure interface
         */
    void dynamicReconfigureUpdate();

    /**
     * @brief _dynamics constraint relates the floating base with the contact forces
     */
    constraints::acceleration::DynamicFeasibility::Ptr _dynamics;

    /**
     * @brief _friction_cones constraints
     */
    constraints::force::FrictionCones::Ptr _friction_cones;

    /**
     * @brief the final ID stack
     */
    AutoStack::Ptr _stack;

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
    std::vector<Eigen::Vector6d> _contact_wrenches;

    Eigen::Vector6d _wrench_upper_lims;
    Eigen::Vector6d _wrench_lower_lims;

    /** @brief ROS dynamic reconfigure */
    dynamic_reconfigure::Server<wb_controller::problemConfig>* server_;
    /** @brief ROS dynamic reconfigure config struct */
    wb_controller::problemConfig default_config_;

    std::atomic<double> _mu;
    std::atomic<double> _x_force_lower_lim;
    std::atomic<double> _y_force_lower_lim;
    std::atomic<double> _z_force_lower_lim;

    unsigned int idx_grfs_start_;

};



} // ID_PROBLEM_H

#endif

