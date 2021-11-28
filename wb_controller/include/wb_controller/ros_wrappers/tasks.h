#ifndef ROS_WRAPPERS_TASKS_H
#define ROS_WRAPPERS_TASKS_H

// OpenSoT
#include <OpenSoT/Task.h>
#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <OpenSoT/tasks/acceleration/CoM.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/tasks/acceleration/AngularMomentum.h>

// ROS
#include <wb_controller/CartesianTask.h>
#include <wb_controller/ComTask.h>
#include <wb_controller/PosturalTask.h>

// WB
#include <wb_controller/ros_wrappers/interface.h>
#include <wb_controller/geometry.h>
#include <wb_controller/utils.h>

// CARTESIAN
class Cartesian : public OpenSoT::tasks::acceleration::Cartesian, public TaskRosWrapperInterface<wb_controller::CartesianTask>
{

public:

  typedef std::shared_ptr<Cartesian> Ptr;

  Cartesian(ros::NodeHandle& nh,
            const std::string task_id,
            const XBot::ModelInterface& robot,
            const std::string& distal_link,
            const std::string& base_link,
            const OpenSoT::AffineHelper& qddot)
    :OpenSoT::tasks::acceleration::Cartesian(task_id,robot,distal_link,base_link,qddot)
    ,TaskRosWrapperInterface<wb_controller::CartesianTask>(task_id,nh)
  {
    Eigen::Affine3d actual_pose;
    getActualPose(actual_pose);
    buffer_pose_reference_.initRT(actual_pose);
    // Create the interactive marker
    marker_server_.reset(new interactive_markers::InteractiveMarkerServer(_task_id));
    visualization_msgs::InteractiveMarker&& marker = createInteractiveMarker(actual_pose,getBaseLink());
    marker_server_->insert(marker,boost::bind(&Cartesian::processFeedback, this, _1));
    marker_server_->applyChanges();

    // Setup the filters
    position_reference_filter_.setOmega(2.0*M_PI*20.0); // 20Hz cutoff FIXME hardcoded
    position_reference_filter_.setTimeStep(wb_controller::_period);
  }

  virtual void registerReconfigurableVariables() override
  {
    double lambda1 = getLambda();
    double lambda2 = getLambda2();
    double weight  = getWeight()(0,0);
    server_->registerVariable<double>("set_lambda_1",    lambda1,     boost::bind(&TaskRosWrapperInterface::setLambda1,this,_1)    ,"set lambda 1"   ,0.0,1000.0);
    server_->registerVariable<double>("set_lambda_2",    lambda2,     boost::bind(&TaskRosWrapperInterface::setLambda2,this,_1)    ,"set lambda 2"   ,0.0,1000.0);
    server_->registerVariable<double>("set_weight_diag", weight,      boost::bind(&TaskRosWrapperInterface::setWeightDiag,this,_1) ,"set weight diag",0.0,1000.0);
    Eigen::Matrix6d Kp = getKp();
    server_->registerVariable<double>("kp_x",            Kp(0,0), boost::bind(&TaskRosWrapperInterface::setKpX,this,_1)            ,"Kp(0,0)", 0.0, 1000.0);
    server_->registerVariable<double>("kp_y",            Kp(1,1), boost::bind(&TaskRosWrapperInterface::setKpY,this,_1)            ,"Kp(1,1)", 0.0, 1000.0);
    server_->registerVariable<double>("kp_z",            Kp(2,2), boost::bind(&TaskRosWrapperInterface::setKpZ,this,_1)            ,"Kp(2,2)", 0.0, 1000.0);
    server_->registerVariable<double>("kp_roll",         Kp(3,3), boost::bind(&TaskRosWrapperInterface::setKpRoll,this,_1)         ,"Kp(3,3)", 0.0, 1000.0);
    server_->registerVariable<double>("kp_pitch",        Kp(4,4), boost::bind(&TaskRosWrapperInterface::setKpPitch,this,_1)        ,"Kp(4,4)", 0.0, 1000.0);
    server_->registerVariable<double>("kp_yaw",          Kp(5,5), boost::bind(&TaskRosWrapperInterface::setKpYaw,this,_1)          ,"Kp(5,5)", 0.0, 1000.0);
    Eigen::Matrix6d Kd = getKd();
    server_->registerVariable<double>("kd_x",            Kd(0,0), boost::bind(&TaskRosWrapperInterface::setKdX,this,_1)            ,"Kd(0,0)", 0.0, 1000.0);
    server_->registerVariable<double>("kd_y",            Kd(1,1), boost::bind(&TaskRosWrapperInterface::setKdY,this,_1)            ,"Kd(1,1)", 0.0, 1000.0);
    server_->registerVariable<double>("kd_z",            Kd(2,2), boost::bind(&TaskRosWrapperInterface::setKdZ,this,_1)            ,"Kd(2,2)", 0.0, 1000.0);
    server_->registerVariable<double>("kd_roll",         Kd(3,3), boost::bind(&TaskRosWrapperInterface::setKdRoll,this,_1)         ,"Kd(3,3)", 0.0, 1000.0);
    server_->registerVariable<double>("kd_pitch",        Kd(4,4), boost::bind(&TaskRosWrapperInterface::setKdPitch,this,_1)        ,"Kd(4,4)", 0.0, 1000.0);
    server_->registerVariable<double>("kd_yaw",          Kd(5,5), boost::bind(&TaskRosWrapperInterface::setKdYaw,this,_1)          ,"Kd(5,5)", 0.0, 1000.0);
    server_->publishServicesTopics();
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

    Eigen::Matrix6d Kp = Eigen::Matrix6d::Zero();
    Eigen::Matrix6d Kd = Eigen::Matrix6d::Zero();
    bool use_identity = false;
    for(unsigned int i=0; i<wb_controller::_cartesian_names.size(); i++)
    {
      if (!nh_.getParam("gains/"+_task_id+"/Kp/" + wb_controller::_cartesian_names[i] , Kp(i,i)))
      {
        ROS_WARN("No Kp.%s gain given for task %s in the namespace: %s, using an identity matrix. ",wb_controller::_cartesian_names[i].c_str(),_task_id.c_str(),nh_.getNamespace().c_str());
        use_identity = true;
      }
      if (!nh_.getParam("gains/"+_task_id+"/Kd/"  + wb_controller::_cartesian_names[i] , Kd(i,i)))
      {
        ROS_WARN("No Kd.%s gain given for task %s in the namespace: %s, using an identity matrix. ",wb_controller::_cartesian_names[i].c_str(),_task_id.c_str(),nh_.getNamespace().c_str());
        use_identity = true;
      }
      // Check if the values are positive
      if(Kp(i,i)<0.0 || Kd(i,i)<0.0)
        throw std::runtime_error("Kp and Kd must be positive definite!");

    }

    if(use_identity)
    {
      Kp = Eigen::Matrix6d::Identity();
      Kd = Eigen::Matrix6d::Identity();
    }

    buffer_kp_x_     = Kp(0,0);
    buffer_kp_y_     = Kp(1,1);
    buffer_kp_z_     = Kp(2,2);
    buffer_kp_roll_  = Kp(3,3);
    buffer_kp_pitch_ = Kp(4,4);
    buffer_kp_yaw_   = Kp(5,5);

    buffer_kd_x_     = Kd(0,0);
    buffer_kd_y_     = Kd(1,1);
    buffer_kd_z_     = Kd(2,2);
    buffer_kd_roll_  = Kd(3,3);
    buffer_kd_pitch_ = Kd(4,4);
    buffer_kd_yaw_   = Kd(5,5);

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
      getActualPose(tmp_affine3d_);
      getActualTwist(tmp_vector6d_);
      wb_controller::rotTorpy(tmp_affine3d_.linear(),tmp_vector3d_);
      wb_controller::affine3dToPose(tmp_affine3d_,rt_pub_->msg_.pose_actual);
      wb_controller::vector6dToTwist(tmp_vector6d_,rt_pub_->msg_.twist_actual);
      wb_controller::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.rpy_actual);

      // REFERENCE VALUES
      getReference(tmp_affine3d_);
      tmp_vector6d_ = getCachedVelocityReference();
      wb_controller::rotTorpy(tmp_affine3d_.linear(),tmp_vector3d_);
      wb_controller::affine3dToPose(tmp_affine3d_,rt_pub_->msg_.pose_reference);
      wb_controller::vector6dToTwist(tmp_vector6d_,rt_pub_->msg_.twist_reference);
      wb_controller::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.rpy_reference);

      // COST
      rt_pub_->msg_.cost = cost_;

      // PUBLISH
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
      tmp_matrix6d_.setZero();
      tmp_matrix6d_(0,0) = buffer_kp_x_;
      tmp_matrix6d_(1,1) = buffer_kp_y_;
      tmp_matrix6d_(2,2) = buffer_kp_z_;
      tmp_matrix6d_(3,3) = buffer_kp_roll_;
      tmp_matrix6d_(4,4) = buffer_kp_pitch_;
      tmp_matrix6d_(5,5) = buffer_kp_yaw_;
      setKp(tmp_matrix6d_);
      tmp_matrix6d_.setZero();
      tmp_matrix6d_(0,0) = buffer_kd_x_;
      tmp_matrix6d_(1,1) = buffer_kd_y_;
      tmp_matrix6d_(2,2) = buffer_kd_z_;
      tmp_matrix6d_(3,3) = buffer_kd_roll_;
      tmp_matrix6d_(4,4) = buffer_kd_pitch_;
      tmp_matrix6d_(5,5) = buffer_kd_yaw_;
      setKd(tmp_matrix6d_);
    }
    if(OPTIONS.set_ext_reference)
    {
      tmp_affine3d_.setIdentity();
      tmp_affine3d_ = *buffer_pose_reference_.readFromRT();
      // Filter
      tmp_affine3d_.translation() = position_reference_filter_.process(tmp_affine3d_.translation());
      setReference(tmp_affine3d_);
    }
    OpenSoT::tasks::acceleration::Cartesian::_update(x);
  }

  virtual bool reset() override
  {
    bool res = OpenSoT::tasks::acceleration::Cartesian::reset();
    getActualPose(tmp_affine3d_);
    buffer_pose_reference_.writeFromNonRT(tmp_affine3d_);
    marker_server_->clear();
    marker_server_->applyChanges();
    visualization_msgs::InteractiveMarker&& marker = createInteractiveMarker(tmp_affine3d_,getBaseLink());
    marker_server_->insert(marker,boost::bind(&Cartesian::processFeedback, this, _1));
    marker_server_->applyChanges();
    return res;
  }

  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
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

    buffer_pose_reference_.writeFromNonRT(pose_reference);
  }

private:

  visualization_msgs::InteractiveMarker createInteractiveMarker(const Eigen::Affine3d& initial_pose, const std::string& frame)
  {

    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = frame;
    int_marker.name = _task_id;
    int_marker.description = _task_id;
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

};

// CoM
class CoM : public OpenSoT::tasks::acceleration::CoM, public TaskRosWrapperInterface<wb_controller::ComTask>
{

public:

  typedef std::shared_ptr<CoM> Ptr;

  CoM(ros::NodeHandle& nh, const XBot::ModelInterface& robot, const OpenSoT::AffineHelper& qddot)
    :OpenSoT::tasks::acceleration::CoM(robot,qddot)
    ,TaskRosWrapperInterface<wb_controller::ComTask>(_task_id,nh)
  {
  }

  virtual void registerReconfigurableVariables() override
  {
    double lambda1 = getLambda();
    double lambda2 = getLambda2();
    double weight  = getWeight()(0,0);
    server_->registerVariable<double>("set_lambda_1",    lambda1,     boost::bind(&TaskRosWrapperInterface::setLambda1,this,_1)    ,"set lambda 1"   ,0.0,1000.0);
    server_->registerVariable<double>("set_lambda_2",    lambda2,     boost::bind(&TaskRosWrapperInterface::setLambda2,this,_1)    ,"set lambda 2"   ,0.0,1000.0);
    server_->registerVariable<double>("set_weight_diag", weight,      boost::bind(&TaskRosWrapperInterface::setWeightDiag,this,_1) ,"set weight diag",0.0,1000.0);
    Eigen::Matrix3d Kp = getKp();
    server_->registerVariable<double>("kp_x",            Kp(0,0), boost::bind(&TaskRosWrapperInterface::setKpX,this,_1)            ,"Kp(0,0)", 0.0, 1000.0);
    server_->registerVariable<double>("kp_y",            Kp(1,1), boost::bind(&TaskRosWrapperInterface::setKpY,this,_1)            ,"Kp(1,1)", 0.0, 1000.0);
    server_->registerVariable<double>("kp_z",            Kp(2,2), boost::bind(&TaskRosWrapperInterface::setKpZ,this,_1)            ,"Kp(2,2)", 0.0, 1000.0);
    Eigen::Matrix3d Kd = getKd();
    server_->registerVariable<double>("kd_x",            Kd(0,0), boost::bind(&TaskRosWrapperInterface::setKdX,this,_1)            ,"Kd(0,0)", 0.0, 1000.0);
    server_->registerVariable<double>("kd_y",            Kd(1,1), boost::bind(&TaskRosWrapperInterface::setKdY,this,_1)            ,"Kd(1,1)", 0.0, 1000.0);
    server_->registerVariable<double>("kd_z",            Kd(2,2), boost::bind(&TaskRosWrapperInterface::setKdZ,this,_1)            ,"Kd(2,2)", 0.0, 1000.0);
    server_->publishServicesTopics();
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
    for(unsigned int i=0; i<wb_controller::_xyz.size(); i++)
    {
      if (!nh_.getParam("gains/"+_task_id+"/Kp/" + wb_controller::_xyz[i] , Kp(i,i)))
      {
        ROS_WARN("No Kp.%s gain given for task %s in the namespace: %s, using an identity matrix. ",wb_controller::_xyz[i].c_str(),_task_id.c_str(),nh_.getNamespace().c_str());
        use_identity = true;
      }
      if (!nh_.getParam("gains/"+_task_id+"/Kd/"  + wb_controller::_xyz[i] , Kd(i,i)))
      {
        ROS_WARN("No Kd.%s gain given for task %s in the namespace: %s, using an identity matrix. ",wb_controller::_xyz[i].c_str(),_task_id.c_str(),nh_.getNamespace().c_str());
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
      wb_controller::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.position_actual);
      // Velocity reference
      tmp_vector3d_ = getCachedVelocityReference();
      wb_controller::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.velocity_reference);

      // REFERENCE VALUES
      getReference(tmp_vector3d_);
      // Pose - Translation
      wb_controller::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.position_reference);

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
    OpenSoT::tasks::acceleration::CoM::_update(x);
  }

};

// POSTURAL
class Postural : public OpenSoT::tasks::acceleration::Postural, public TaskRosWrapperInterface<wb_controller::PosturalTask>
{

public:

  typedef std::shared_ptr<Postural> Ptr;

  Postural(ros::NodeHandle& nh, const XBot::ModelInterface& robot,
           OpenSoT::AffineHelper qddot = OpenSoT::AffineHelper(), const std::string task_id = "Postural")
    :OpenSoT::tasks::acceleration::Postural(robot,qddot,task_id)
    ,TaskRosWrapperInterface<wb_controller::PosturalTask>(task_id,nh)
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
    server_->publishServicesTopics();
  }

  virtual void registerReconfigurableVariables() override
  {
    double lambda1 = getLambda();
    double lambda2 = getLambda2();
    double weight  = getWeight()(0,0);
    server_->registerVariable<double>("set_lambda_1",    lambda1,     boost::bind(&TaskRosWrapperInterface::setLambda1,this,_1)    ,"set lambda 1"   ,0.0,1000.0);
    server_->registerVariable<double>("set_lambda_2",    lambda2,     boost::bind(&TaskRosWrapperInterface::setLambda2,this,_1)    ,"set lambda 2"   ,0.0,1000.0);
    server_->registerVariable<double>("set_weight_diag", weight,      boost::bind(&TaskRosWrapperInterface::setWeightDiag,this,_1) ,"set weight diag",0.0,1000.0);
    server_->publishServicesTopics();
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
        rt_pub_->msg_.name[i] = wb_controller::_dof_names[i];
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

// AngularMomentum
class AngularMomentum : public OpenSoT::tasks::acceleration::AngularMomentum, public TaskRosWrapperInterface<wb_controller::CartesianTask>
{

public:

  typedef std::shared_ptr<AngularMomentum> Ptr;

  AngularMomentum(ros::NodeHandle& nh, XBot::ModelInterface& robot, const OpenSoT::AffineHelper& qddot)
    :OpenSoT::tasks::acceleration::AngularMomentum(robot,qddot)
    ,TaskRosWrapperInterface<wb_controller::CartesianTask>(_task_id,nh)
  {
  }

  virtual void registerReconfigurableVariables() override
  {
    double lambda1 = getLambda();
    double weight  = getWeight()(0,0);
    server_->registerVariable<double>("set_lambda_1",    lambda1,     boost::bind(&TaskRosWrapperInterface::setLambda1,this,_1)    ,"set lambda 1"   ,0.0,1000.0);
    server_->registerVariable<double>("set_weight_diag", weight,      boost::bind(&TaskRosWrapperInterface::setWeightDiag,this,_1) ,"set weight diag",0.0,1000.0);
    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    server_->registerVariable<double>("K_roll",    K(0,0), boost::bind(&TaskRosWrapperInterface::setKpRoll, this,_1)    ,"K(0,0)", 0.0, 1000.0);
    server_->registerVariable<double>("K_pitch",   K(1,1), boost::bind(&TaskRosWrapperInterface::setKpPitch,this,_1)    ,"K(1,1)", 0.0, 1000.0);
    server_->registerVariable<double>("K_yaw",     K(2,2), boost::bind(&TaskRosWrapperInterface::setKpYaw,  this,_1)    ,"K(2,2)", 0.0, 1000.0);
    server_->publishServicesTopics();
  }

  virtual void loadParams() override
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
    for(unsigned int i=0; i<wb_controller::_rpy.size(); i++)
    {
      if (!nh_.getParam("gains/"+_task_id+"/K/" + wb_controller::_rpy[i] , K(i,i)))
      {
        ROS_WARN("No Kp.%s gain given for task %s in the namespace: %s, using an identity matrix. ",wb_controller::_rpy[i].c_str(),_task_id.c_str(),nh_.getNamespace().c_str());
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

  virtual void updateCost(const Eigen::VectorXd& x) override
  {
    cost_ = computeCost(x);
  }

  virtual void update(const Eigen::VectorXd& x)
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

  virtual void publish(const ros::Time& time)
  {

  }

};

#endif // ROS_WRAPPERS_TASKS_H

