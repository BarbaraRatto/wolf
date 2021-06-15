#ifndef TASK_ROS_WRAPPER_H
#define TASK_ROS_WRAPPER_H

// OpenSoT
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <OpenSoT/tasks/acceleration/CoM.h>
#include <OpenSoT/tasks/MinimizeVariable.h>
#include <OpenSoT/tasks/acceleration/DynamicFeasibility.h>
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
#include <wb_controller/taskGenericConfig.h>
#include <wb_controller/taskPosturalConfig.h>
#include <wb_controller/taskCartesianConfig.h>

// WB
#include <wb_controller/geometry.h>
#include <wb_controller/utils.h>

// STD
#include <atomic>

class TaskRosWrapperInterface
{

public:

    typedef std::shared_ptr<TaskRosWrapperInterface> Ptr;

    TaskRosWrapperInterface(){spinner_.reset(new ros::AsyncSpinner(1)); spinner_->start();}
    virtual ~TaskRosWrapperInterface(){spinner_->stop();}

    virtual void publish(const ros::Time& /*time*/) = 0;
    virtual void updateReference() {}
    virtual void dynamicReconfigureUpdate() {}

protected:

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;

    Eigen::Affine3d       tmp_affine3d_;
    Eigen::VectorXd       tmp_vectorxd_;
    Eigen::Vector3d       tmp_vector3d_;
    Eigen::Vector6d       tmp_vector6d_;
    Eigen::Quaterniond    tmp_quaterniond_;

    realtime_tools::RealtimeBuffer<Eigen::Affine3d> rt_affine3d_;

};

template <typename task_ptr_t, typename msg_t, typename config_t>
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
        server_.reset(new dynamic_reconfigure::Server<config_t>(task_nh));
        server_->setCallback(boost::bind(&TaskRosWrapperBase::dynamicReconfigureCallback, this, _1, _2));
    }

    virtual void dynamicReconfigureCallback(config_t &config, uint32_t level)
    {
        switch(level)
        {
        case 0:
            task_->setLambda(config.lambda1,config.lambda2);
            break;
        case 1:
            if(config.weight_diag>=0)
            {
                //task_->setWeight(Eigen::MatrixXd::Identity(task_->getTaskSize(),task_->getTaskSize()) * config.weight_diag);
                task_->setWeight(config.weight_diag);
                ROS_INFO_STREAM("Set weight diagonal values for task "<<task_->getTaskID()<<" to "<<task_->getWeight()(0,0));
            }
            else
                ROS_WARN("Weight diagonal value has to be positive definite!");
            break;
        }
    }

    virtual void dynamicReconfigureUpdate()
    {
        default_config_.lambda1 = task_->getLambda();
        default_config_.lambda2 = task_->getLambda2();
        default_config_.weight_diag = task_->getWeight()(0,0); // Note: we take the first value of the diagonal.
        if(server_) server_->updateConfig(default_config_);
    }

    virtual void publish(const ros::Time& /*time*/) = 0;

protected:

    std::shared_ptr<dynamic_reconfigure::Server<config_t>> server_;
    config_t default_config_;
    std::shared_ptr<realtime_tools::RealtimePublisher<msg_t>> rt_pub_;
    task_ptr_t task_;
};

template <typename task_ptr_t, typename msg_t, typename config_t>
class TaskRosWrapper : public TaskRosWrapperBase<task_ptr_t,msg_t,config_t>
{
public:
    TaskRosWrapper(ros::NodeHandle& nh, task_ptr_t task)
    {
        TaskRosWrapperBase<task_ptr_t,msg_t,config_t>(nh,task);
    }
};

// Specializations:

// CARTESIAN - CARTESIAN
template <>
class TaskRosWrapper<OpenSoT::tasks::acceleration::Cartesian::Ptr,wb_controller::CartesianTask,wb_controller::taskCartesianConfig>
        : public TaskRosWrapperBase<OpenSoT::tasks::acceleration::Cartesian::Ptr,wb_controller::CartesianTask,wb_controller::taskCartesianConfig>
{

public:

    TaskRosWrapper(ros::NodeHandle& nh, OpenSoT::tasks::acceleration::Cartesian::Ptr task):
        TaskRosWrapperBase<OpenSoT::tasks::acceleration::Cartesian::Ptr,wb_controller::CartesianTask,wb_controller::taskCartesianConfig>(nh,task)
    {

        task_->getActualPose(tmp_affine3d_);

        rt_affine3d_.initRT(tmp_affine3d_);

        tmp_quaterniond_ = tmp_affine3d_.linear();

        createInteractiveMarker(tmp_affine3d_);

        // Load params
        Eigen::Matrix6d Kp = Eigen::Matrix6d::Zero();
        Eigen::Matrix6d Kd = Eigen::Matrix6d::Zero();
        bool use_identity = false;
        for(unsigned int i=0; i<wb_controller::_cartesian_names.size(); i++)
        {
            if (!nh.getParam("gains/"+task_->getTaskID()+"/Kp/" + wb_controller::_cartesian_names[i] , Kp(i,i)))
            {
                ROS_WARN("No Kp.%s gain given for task %s in the namespace: %s, using an identity matrix. ",wb_controller::_cartesian_names[i].c_str(),task_->getTaskID().c_str(),nh.getNamespace().c_str());
                use_identity = true;
            }
            if (!nh.getParam("gains/"+task_->getTaskID()+"/Kd/"  + wb_controller::_cartesian_names[i] , Kd(i,i)))
            {
                ROS_WARN("No Kd.%s gain given for task %s in the namespace: %s, using an identity matrix. ",wb_controller::_cartesian_names[i].c_str(),task_->getTaskID().c_str(),nh.getNamespace().c_str());
                use_identity = true;
            }
            // Check if the values are positive
            if(Kp(i,i)<0.0 || Kd(i,i)<0.0)
            {
                ROS_WARN("Kp and Kd gains must be positive!");
                use_identity = true;
            }
        }

        if(use_identity)
        {
          Kp = Eigen::Matrix6d::Identity();
          Kd = Eigen::Matrix6d::Identity();
        }

        task_->setGains(Kp,Kd);
    }

    virtual void publish(const ros::Time& time) override
    {
        if(rt_pub_->trylock())
        {
            rt_pub_->msg_.header.frame_id = task_->getBaseLink();
            rt_pub_->msg_.header.stamp = time;

            // ACTUAL VALUES
            task_->getActualPose(tmp_affine3d_);
            task_->getActualTwist(tmp_vector6d_);
            wb_controller::rotTorpy(tmp_affine3d_.linear(),tmp_vector3d_);
            wb_controller::affine3dToPose(tmp_affine3d_,rt_pub_->msg_.pose_actual);
            wb_controller::vector6dToTwist(tmp_vector6d_,rt_pub_->msg_.twist_actual);
            wb_controller::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.rpy_actual);

            // REFERENCE VALUES
            task_->getReference(tmp_affine3d_);
            tmp_vector6d_ = task_->getCachedVelocityReference();
            wb_controller::rotTorpy(tmp_affine3d_.linear(),tmp_vector3d_);
            wb_controller::affine3dToPose(tmp_affine3d_,rt_pub_->msg_.pose_reference);
            wb_controller::vector6dToTwist(tmp_vector6d_,rt_pub_->msg_.twist_reference);
            wb_controller::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.rpy_reference);

            rt_pub_->unlockAndPublish();
        }
    }

    virtual void updateReference()
    {
        // Set the task reference for the cartesian task
        task_->setReference(*rt_affine3d_.readFromRT());
    }

    virtual void dynamicReconfigureCallback(wb_controller::taskCartesianConfig &config, uint32_t level)
    {
        TaskRosWrapperBase::dynamicReconfigureCallback(config,level);
        switch(level)
        {
        case 2:
            Eigen::Matrix6d Kp = Eigen::Matrix6d::Zero();
            Eigen::Matrix6d Kd = Eigen::Matrix6d::Zero();

            Kp(0,0) = config.kp_x;
            Kp(1,1) = config.kp_y;
            Kp(2,2) = config.kp_z;
            Kp(3,3) = config.kp_roll;
            Kp(4,4) = config.kp_pitch;
            Kp(5,5) = config.kp_yaw;

            Kd(0,0) = config.kd_x;
            Kd(1,1) = config.kd_y;
            Kd(2,2) = config.kd_z;
            Kd(3,3) = config.kd_roll;
            Kd(4,4) = config.kd_pitch;
            Kd(5,5) = config.kd_yaw;

            task_->setGains(Kp,Kd);

            ROS_INFO_STREAM("Set Kp and Kd for task: "<<task_->getTaskID());

            break;
        }
    }

    virtual void dynamicReconfigureUpdate()
    {

        default_config_.kp_x     = task_->getKp()(0,0);
        default_config_.kp_y     = task_->getKp()(1,1);
        default_config_.kp_z     = task_->getKp()(2,2);
        default_config_.kp_roll  = task_->getKp()(3,3);
        default_config_.kp_pitch = task_->getKp()(4,4);
        default_config_.kp_yaw   = task_->getKp()(5,5);

        default_config_.kd_x     = task_->getKd()(0,0);
        default_config_.kd_y     = task_->getKd()(1,1);
        default_config_.kd_z     = task_->getKd()(2,2);
        default_config_.kd_roll  = task_->getKd()(3,3);
        default_config_.kd_pitch = task_->getKd()(4,4);
        default_config_.kd_yaw   = task_->getKd()(5,5);

        TaskRosWrapperBase::dynamicReconfigureUpdate();
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
        pose_reference.linear() = R.transpose();

        /*quaternion.x() = feedback->pose.orientation.x;
        quaternion.y() = feedback->pose.orientation.y;
        quaternion.z() = feedback->pose.orientation.z;
        quaternion.w() = feedback->pose.orientation.w;
        quatToRotMat(quaternion,world_R_task);*/

        // FIXME no orientation yet
        rt_affine3d_.writeFromNonRT(pose_reference);
    }

private:

    void createInteractiveMarker(const Eigen::Affine3d& initial_pose)
    {

      marker_.reset(new interactive_markers::InteractiveMarkerServer(task_->getTaskID()));

      // create an interactive marker for our server
      visualization_msgs::InteractiveMarker int_marker;
      int_marker.header.frame_id = task_->getBaseLink();
      int_marker.name = task_->getTaskID();
      int_marker.description = task_->getTaskID();

      wb_controller::affine3dToPose(initial_pose,int_marker.pose);

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

      //control.orientation_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_3D;

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

    }
};

// COM - GENERIC
template <>
class TaskRosWrapper<OpenSoT::tasks::acceleration::CoM::Ptr,wb_controller::CartesianTask,wb_controller::taskGenericConfig>
        : public TaskRosWrapperBase<OpenSoT::tasks::acceleration::CoM::Ptr,wb_controller::CartesianTask,wb_controller::taskGenericConfig>
{

public:

    TaskRosWrapper(ros::NodeHandle& nh, OpenSoT::tasks::acceleration::CoM::Ptr task):
        TaskRosWrapperBase<OpenSoT::tasks::acceleration::CoM::Ptr,wb_controller::CartesianTask,wb_controller::taskGenericConfig>(nh,task){}

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

// POSTURAL - GENERIC
template <>
class TaskRosWrapper<OpenSoT::tasks::acceleration::Postural::Ptr,wb_controller::JointsTask,wb_controller::taskGenericConfig>
        : public TaskRosWrapperBase<OpenSoT::tasks::acceleration::Postural::Ptr,wb_controller::JointsTask,wb_controller::taskGenericConfig>
{

public:

    TaskRosWrapper(ros::NodeHandle& nh, OpenSoT::tasks::acceleration::Postural::Ptr task):
        TaskRosWrapperBase<OpenSoT::tasks::acceleration::Postural::Ptr,wb_controller::JointsTask,wb_controller::taskGenericConfig>(nh,task)
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

    /*virtual void dynamicReconfigureCallback(wb_controller::taskPosturalConfig &config, uint32_t level)
    {
        TaskRosWrapperBase::dynamicReconfigureCallback(config,level);
        switch(level)
        {
        case 2:
            Eigen::MatrixXd Kp = Eigen::MatrixXd::Zero(task_->getTaskSize(),task_->getTaskSize());
            Eigen::MatrixXd Kd = Eigen::MatrixXd::Zero(task_->getTaskSize(),task_->getTaskSize());

            Eigen::Matrix3d Kp_leg = Eigen::Matrix3d::Zero();
            Eigen::Matrix3d Kd_leg = Eigen::Matrix3d::Zero();

            Kp_leg(0,0) = config.kp_haa;
            Kp_leg(1,1) = config.kp_hfe;
            Kp_leg(2,2) = config.kp_kfe;

            Kd_leg(0,0) = config.kd_haa;
            Kd_leg(1,1) = config.kd_hfe;
            Kd_leg(2,2) = config.kd_kfe;

            for(unsigned int i=0;i<N_LEGS;i++)
            {
                Kp.block<3,3>(FLOATING_BASE_DOFS+3*i,FLOATING_BASE_DOFS+3*i) = Kp_leg;
                Kd.block<3,3>(FLOATING_BASE_DOFS+3*i,FLOATING_BASE_DOFS+3*i) = Kd_leg;
            }

            task_->setGains(Kp,Kd);

            ROS_INFO_STREAM("Set Kp and Kd for task: "<<task_->getTaskID());

            break;
        }
    }

    virtual void dynamicReconfigureUpdate()
    {
        default_config_.kp_haa = task_->getKp()(FLOATING_BASE_DOFS,FLOATING_BASE_DOFS);
        default_config_.kp_hfe = task_->getKp()(FLOATING_BASE_DOFS+1,FLOATING_BASE_DOFS+1);
        default_config_.kp_kfe = task_->getKp()(FLOATING_BASE_DOFS+2,FLOATING_BASE_DOFS+2);

        default_config_.kd_haa = task_->getKd()(FLOATING_BASE_DOFS,FLOATING_BASE_DOFS);
        default_config_.kd_hfe = task_->getKd()(FLOATING_BASE_DOFS+1,FLOATING_BASE_DOFS+1);
        default_config_.kd_kfe = task_->getKd()(FLOATING_BASE_DOFS+2,FLOATING_BASE_DOFS+2);

        TaskRosWrapperBase::dynamicReconfigureUpdate();
    }*/
};

typedef TaskRosWrapper<OpenSoT::tasks::acceleration::Cartesian::Ptr,wb_controller::CartesianTask,wb_controller::taskCartesianConfig> CartesianWrapper;
typedef TaskRosWrapper<OpenSoT::tasks::acceleration::CoM::Ptr,wb_controller::CartesianTask,wb_controller::taskGenericConfig> ComWrapper;
typedef TaskRosWrapper<OpenSoT::tasks::acceleration::Postural::Ptr,wb_controller::JointsTask,wb_controller::taskGenericConfig> PosturalWrapper;

#endif // TASK_ROS_WRAPPER_H

