#ifndef ROS_WRAPPERS_TASKS_H
#define ROS_WRAPPERS_TASKS_H

// OpenSoT
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <OpenSoT/tasks/acceleration/CoM.h>

// ROS
#include <wb_controller/CartesianTask.h>
#include <wb_controller/PosturalTask.h>

// WB
#include <wb_controller/ros_wrappers/interface.h>
#include <wb_controller/geometry.h>
#include <wb_controller/utils.h>

template <typename task_ptr_t, typename msg_t>
class TaskRosWrapperBase : public TaskRosWrapperInterface
{

public:

    TaskRosWrapperBase(ros::NodeHandle& nh, task_ptr_t task)
    {
        assert(task);
        task_ = task;

        task_id_ = task->getTaskID();

        double lambda1, lambda2, weight;
        if (!nh.getParam("gains/"+task_id_+"/lambda1" , lambda1))
        {
            ROS_WARN("No lambda1 gain given for task %s in the namespace: %s, using the default value loaded from the task",task_id_.c_str(),nh.getNamespace().c_str());
            lambda1 = task_->getLambda();
        }
        if (!nh.getParam("gains/"+task_id_+"/lambda2" , lambda2))
        {
            ROS_WARN("No lambda2 gain given for task %s in the namespace: %s, using the default value loaded from the task",task_id_.c_str(),nh.getNamespace().c_str());
            lambda2 = task_->getLambda2();
        }
        if (!nh.getParam("gains/"+task_id_+"/weight" , weight))
        {
            ROS_WARN("No weight gain given for task %s in the namespace: %s, using the default value loaded from the task",task_id_.c_str(),nh.getNamespace().c_str());
            weight = task_->getWeight()(0,0);
        }

        if(lambda1 < 0 || lambda2 < 0 || weight < 0)
          throw std::runtime_error("Lambda and weight must be positive!");


        rt_lambda1_ = lambda1;
        rt_lambda2_ = lambda2;
        rt_weight_diag_ = weight;

        task_->setLambda(lambda1,lambda2);
        task_->setWeight(weight);

        rt_pub_.reset(new realtime_tools::RealtimePublisher<msg_t>(nh,task_id_, 4));

        //ros::NodeHandle task_nh(nh.getNamespace()+"/"+task->getTaskID());
        ros::NodeHandle task_nh(task_id_);
        server_.reset(new ddynamic_reconfigure::DDynamicReconfigure(task_nh));
        server_->registerVariable<double>("set_lambda_1",    rt_lambda1_,     boost::bind(&TaskRosWrapperBase::setLambda1,this,_1) ,    "set lambda 1"   ,0.0,1000.0);
        server_->registerVariable<double>("set_lambda_2",    rt_lambda2_,     boost::bind(&TaskRosWrapperBase::setLambda2,this,_1) ,    "set lambda 2"   ,0.0,1000.0);
        server_->registerVariable<double>("set_weight_diag", rt_weight_diag_, boost::bind(&TaskRosWrapperBase::setWeightDiag,this,_1) , "set weight diag",0.0,1000.0);
    }

    void setLambda1(double value) {rt_lambda1_ = value; ROS_INFO_STREAM(task_id_<<" - "<<"Set lambda1: "<<value);}
    void setLambda2(double value) {rt_lambda2_ = value; ROS_INFO_STREAM(task_id_<<" - "<<"Set lambda2: "<<value);}
    void setWeightDiag(double value) {rt_weight_diag_ = value; ROS_INFO_STREAM(task_id_<<" - "<<"Set weight diagonal: "<<value);}

    virtual void publish(const ros::Time& /*time*/) = 0;

    virtual void update()
    {
        if(OPTIONS.set_ext_lambda)
          task_->setLambda(rt_lambda1_,rt_lambda2_);
        if(OPTIONS.set_ext_weight)
          task_->setWeight(rt_weight_diag_);
    }

    virtual void reset() override
    {
      task_->reset();
    }

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

// Specializations:

// CARTESIAN - CARTESIAN
template <>
class TaskRosWrapper<OpenSoT::tasks::acceleration::Cartesian::Ptr,wb_controller::CartesianTask>
        : public TaskRosWrapperBase<OpenSoT::tasks::acceleration::Cartesian::Ptr,wb_controller::CartesianTask>
{

public:

    TaskRosWrapper(ros::NodeHandle& nh, OpenSoT::tasks::acceleration::Cartesian::Ptr task):
        TaskRosWrapperBase<OpenSoT::tasks::acceleration::Cartesian::Ptr,wb_controller::CartesianTask>(nh,task)
    {

        Eigen::Affine3d actual_pose;
        task_->getActualPose(actual_pose);
        rt_pose_reference_.initRT(actual_pose);

        // Create the interactive marker
        marker_server_.reset(new interactive_markers::InteractiveMarkerServer(task_id_));
        visualization_msgs::InteractiveMarker&& marker = createInteractiveMarker(actual_pose,task_->getBaseLink());
        marker_server_->insert(marker,boost::bind(&TaskRosWrapper::processFeedback, this, _1));
        marker_server_->applyChanges();

        // Load params
        Kp_ = Eigen::Matrix6d::Zero();
        Kd_ = Eigen::Matrix6d::Zero();
        bool use_identity = false;
        for(unsigned int i=0; i<wb_controller::_cartesian_names.size(); i++)
        {
            if (!nh.getParam("gains/"+task_id_+"/Kp/" + wb_controller::_cartesian_names[i] , Kp_(i,i)))
            {
                ROS_WARN("No Kp.%s gain given for task %s in the namespace: %s, using an identity matrix. ",wb_controller::_cartesian_names[i].c_str(),task_id_.c_str(),nh.getNamespace().c_str());
                use_identity = true;
            }
            if (!nh.getParam("gains/"+task_id_+"/Kd/"  + wb_controller::_cartesian_names[i] , Kd_(i,i)))
            {
                ROS_WARN("No Kd.%s gain given for task %s in the namespace: %s, using an identity matrix. ",wb_controller::_cartesian_names[i].c_str(),task_id_.c_str(),nh.getNamespace().c_str());
                use_identity = true;
            }
            // Check if the values are positive
            if(Kp_(i,i)<0.0 || Kd_(i,i)<0.0)
            {
                ROS_WARN("Kp and Kd gains must be positive!");
                use_identity = true;
            }
        }

        if(use_identity)
        {
          Kp_ = Eigen::Matrix6d::Identity();
          Kd_ = Eigen::Matrix6d::Identity();
        }

        task_->setKp(Kp_);
        task_->setKd(Kd_);

        position_reference_filter_.setOmega(2.0*M_PI*20.0); // 20Hz cutoff FIXME hardcoded
        position_reference_filter_.setTimeStep(wb_controller::_period);

        server_->registerVariable<double>("kp_x",     Kp_(0,0), boost::bind(&TaskRosWrapper::setKpX,this,_1)     ,"Kp(0,0)", 0.0, 1000.0);
        server_->registerVariable<double>("kp_y",     Kp_(1,1), boost::bind(&TaskRosWrapper::setKpY,this,_1)     ,"Kp(1,1)", 0.0, 1000.0);
        server_->registerVariable<double>("kp_z",     Kp_(2,2), boost::bind(&TaskRosWrapper::setKpZ,this,_1)     ,"Kp(2,2)", 0.0, 1000.0);
        server_->registerVariable<double>("kp_roll",  Kp_(3,3), boost::bind(&TaskRosWrapper::setKpRoll,this,_1)  ,"Kp(3,3)", 0.0, 1000.0);
        server_->registerVariable<double>("kp_pitch", Kp_(4,4), boost::bind(&TaskRosWrapper::setKpPitch,this,_1) ,"Kp(4,4)", 0.0, 1000.0);
        server_->registerVariable<double>("kp_yaw",   Kp_(5,5), boost::bind(&TaskRosWrapper::setKpYaw,this,_1)   ,"Kp(5,5)", 0.0, 1000.0);

        server_->registerVariable<double>("kd_x",     Kd_(0,0), boost::bind(&TaskRosWrapper::setKdX,this,_1)     ,"Kd(0,0)", 0.0, 1000.0);
        server_->registerVariable<double>("kd_y",     Kd_(1,1), boost::bind(&TaskRosWrapper::setKdY,this,_1)     ,"Kd(1,1)", 0.0, 1000.0);
        server_->registerVariable<double>("kd_z",     Kd_(2,2), boost::bind(&TaskRosWrapper::setKdZ,this,_1)     ,"Kd(2,2)", 0.0, 1000.0);
        server_->registerVariable<double>("kd_roll",  Kd_(3,3), boost::bind(&TaskRosWrapper::setKdRoll,this,_1)  ,"Kd(3,3)", 0.0, 1000.0);
        server_->registerVariable<double>("kd_pitch", Kd_(4,4), boost::bind(&TaskRosWrapper::setKdPitch,this,_1) ,"Kd(4,4)", 0.0, 1000.0);
        server_->registerVariable<double>("kd_yaw",   Kd_(5,5), boost::bind(&TaskRosWrapper::setKdYaw,this,_1)   ,"Kd(5,5)", 0.0, 1000.0);

        server_->publishServicesTopics();
    }

    void setKpX(double value)     { rt_kp_x_     = value; ROS_INFO_STREAM(task_id_<<" - "<<"Set Kp(0,0): "<<value); }
    void setKpY(double value)     { rt_kp_y_     = value; ROS_INFO_STREAM(task_id_<<" - "<<"Set Kp(1,1): "<<value); }
    void setKpZ(double value)     { rt_kp_z_     = value; ROS_INFO_STREAM(task_id_<<" - "<<"Set Kp(2,2): "<<value); }
    void setKpRoll(double value)  { rt_kp_roll_  = value; ROS_INFO_STREAM(task_id_<<" - "<<"Set Kp(3,3): "<<value); }
    void setKpPitch(double value) { rt_kp_pitch_ = value; ROS_INFO_STREAM(task_id_<<" - "<<"Set Kp(4,4): "<<value); }
    void setKpYaw(double value)   { rt_kp_yaw_   = value; ROS_INFO_STREAM(task_id_<<" - "<<"Set Kp(5,5): "<<value); }

    void setKdX(double value)     { rt_kd_x_     = value; ROS_INFO_STREAM(task_id_<<" - "<<"Set Kd(0,0): "<<value); }
    void setKdY(double value)     { rt_kd_y_     = value; ROS_INFO_STREAM(task_id_<<" - "<<"Set Kd(1,1): "<<value); }
    void setKdZ(double value)     { rt_kd_z_     = value; ROS_INFO_STREAM(task_id_<<" - "<<"Set Kd(2,2): "<<value); }
    void setKdRoll(double value)  { rt_kd_roll_  = value; ROS_INFO_STREAM(task_id_<<" - "<<"Set Kd(3,3): "<<value); }
    void setKdPitch(double value) { rt_kd_pitch_ = value; ROS_INFO_STREAM(task_id_<<" - "<<"Set Kd(4,4): "<<value); }
    void setKdYaw(double value)   { rt_kd_yaw_   = value; ROS_INFO_STREAM(task_id_<<" - "<<"Set Kd(5,5): "<<value); }


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

    virtual void update() override
    {
        TaskRosWrapperBase::update();
        if(OPTIONS.set_ext_gains)
          setExternalGains();
        if(OPTIONS.set_ext_reference)
          setExternalReference();
    }

    virtual Eigen::Affine3d& getExternalReference() override
    {
        return *rt_pose_reference_.readFromRT();
    }

    virtual void setExternalReference() override
    {
        tmp_affine3d_ = *rt_pose_reference_.readFromRT();
        // Filter
        tmp_affine3d_.translation() = position_reference_filter_.process(tmp_affine3d_.translation());
        task_->setReference(tmp_affine3d_);
    }

    //void getExternalGains(Eigen::MatrixBase<Eigen::Matrix6d>& Kp, Eigen::MatrixBase<Eigen::Matrix6d>& Kd)
    //{
    //    Kp = task_->getKp();
    //    Kd = task_->getKd();
    //}
    //
    //Eigen::MatrixBase<Eigen::Matrix6d>& getExternalKp()
    //{
    //    return Kp = task_->getKp();
    //}
    //
    //Eigen::MatrixBase<Eigen::Matrix6d>& getExternalKd()
    //{
    //    return Kd = task_->getKd();
    //}

    virtual void setExternalGains() override
    {
        Kp_(0,0) = rt_kp_x_;
        Kp_(1,1) = rt_kp_y_;
        Kp_(2,2) = rt_kp_z_;
        Kp_(3,3) = rt_kp_roll_;
        Kp_(4,4) = rt_kp_pitch_;
        Kp_(5,5) = rt_kp_yaw_;

        Kd_(0,0) = rt_kd_x_;
        Kd_(1,1) = rt_kd_y_;
        Kd_(2,2) = rt_kd_z_;
        Kd_(3,3) = rt_kd_roll_;
        Kd_(4,4) = rt_kd_pitch_;
        Kd_(5,5) = rt_kd_yaw_;

        task_->setGains(Kp_,Kd_);
    }

    virtual void reset() override
    {
        TaskRosWrapperBase::reset();

        Eigen::Affine3d pose;
        task_->getActualPose(pose);
        rt_pose_reference_.writeFromNonRT(pose);

        marker_server_->clear();
        marker_server_->applyChanges();
        visualization_msgs::InteractiveMarker&& marker = createInteractiveMarker(pose,task_->getBaseLink());
        marker_server_->insert(marker,boost::bind(&TaskRosWrapper::processFeedback, this, _1));
        marker_server_->applyChanges();
    }

protected:

    virtual void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        ROS_DEBUG_STREAM( feedback->marker_name << " is now at "
                         << feedback->pose.position.x << ", " << feedback->pose.position.y
                         << ", " << feedback->pose.position.z << " frame " << feedback->header.frame_id );

        Eigen::Vector3d translation_reference(feedback->pose.position.x,feedback->pose.position.y,feedback->pose.position.z);
        Eigen::Quaterniond orientation_reference(feedback->pose.orientation.w,feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z);
        Eigen::Affine3d pose_reference = Eigen::Affine3d::Identity();
        Eigen::Matrix3d R;

        wb_controller::quatToRotMat(orientation_reference,R);

        pose_reference.translation() = translation_reference;
        pose_reference.linear() = R.transpose();

        rt_pose_reference_.writeFromNonRT(pose_reference);
    }

private:

    visualization_msgs::InteractiveMarker createInteractiveMarker(const Eigen::Affine3d& initial_pose, const std::string& frame)
    {

      // create an interactive marker for our server
      visualization_msgs::InteractiveMarker int_marker;
      int_marker.header.frame_id = frame;
      int_marker.name = task_id_;
      int_marker.description = task_id_;
      int_marker.scale = 0.25;

      wb_controller::affine3dToPose(initial_pose,int_marker.pose);

      // create a grey box marker
      visualization_msgs::Marker box_marker;
      box_marker.type = visualization_msgs::Marker::SPHERE;
      box_marker.scale.x = box_marker.scale.y = box_marker.scale.z = 0.05;
      box_marker.color.r = box_marker.color.g = box_marker.color.b = box_marker.color.a = 0.5 ;

      // create a non-interactive control which contains the box
      visualization_msgs::InteractiveMarkerControl box_control;
      box_control.always_visible = true;
      box_control.markers.push_back( box_marker );

      // add the control to the interactive marker
      int_marker.controls.push_back( box_control );

      visualization_msgs::InteractiveMarkerControl control;

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
      //marker_->insert(int_marker, );
      //
      //// 'commit' changes and send to all clients
      //marker_->applyChanges();
      return int_marker;

    }

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
    visualization_msgs::InteractiveMarker marker_;

    XBot::Utils::SecondOrderFilter<Eigen::Vector3d> position_reference_filter_;

    Eigen::Matrix6d Kp_;
    Eigen::Matrix6d Kd_;
};

// COM - GENERIC
template <>
class TaskRosWrapper<OpenSoT::tasks::acceleration::CoM::Ptr,wb_controller::CartesianTask>
        : public TaskRosWrapperBase<OpenSoT::tasks::acceleration::CoM::Ptr,wb_controller::CartesianTask>
{

public:

    TaskRosWrapper(ros::NodeHandle& nh, OpenSoT::tasks::acceleration::CoM::Ptr task):
        TaskRosWrapperBase<OpenSoT::tasks::acceleration::CoM::Ptr,wb_controller::CartesianTask>(nh,task){ server_->publishServicesTopics(); }

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
            // Velocity reference
            tmp_vector3d_ = task_->getCachedVelocityReference();
            wb_controller::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.twist_reference.linear);

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
class TaskRosWrapper<OpenSoT::tasks::acceleration::Postural::Ptr,wb_controller::PosturalTask>
        : public TaskRosWrapperBase<OpenSoT::tasks::acceleration::Postural::Ptr,wb_controller::PosturalTask>
{

public:

    TaskRosWrapper(ros::NodeHandle& nh, OpenSoT::tasks::acceleration::Postural::Ptr task):
        TaskRosWrapperBase<OpenSoT::tasks::acceleration::Postural::Ptr,wb_controller::PosturalTask>(nh,task)
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

        server_->publishServicesTopics();
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

            ROS_INFO_STREAM("Set Kp and Kd for task: "<<task_id_);

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

typedef TaskRosWrapper<OpenSoT::tasks::acceleration::Cartesian::Ptr,wb_controller::CartesianTask> CartesianWrapper;
typedef TaskRosWrapper<OpenSoT::tasks::acceleration::CoM::Ptr,wb_controller::CartesianTask> ComWrapper;
typedef TaskRosWrapper<OpenSoT::tasks::acceleration::Postural::Ptr,wb_controller::PosturalTask> PosturalWrapper;

#endif // ROS_WRAPPERS_TASKS_H

