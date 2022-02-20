/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef ROS_WRAPPERS_CARTESIAN_H
#define ROS_WRAPPERS_CARTESIAN_H

// OpenSoT
#include <OpenSoT/Task.h>
#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <OpenSoT/tasks/acceleration/CoM.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/tasks/acceleration/AngularMomentum.h>

// ROS
#include <wolf_controller/CartesianTask.h>
#include <wolf_controller/ComTask.h>
#include <wolf_controller/PosturalTask.h>
#include <interactive_markers/menu_handler.h>
#include <urdf/model.h>
#include <kdl_conversions/kdl_msg.h>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseArray.h>

// WoLF
#include <wolf_controller/ros_wrappers/interface.h>
#include <wolf_controller/geometry.h>
#include <wolf_controller/utils.h>

// STD
#include <numeric>

#define SECS 5

// CARTESIAN
class Cartesian : public OpenSoT::tasks::acceleration::Cartesian, public TaskRosWrapperInterface<wolf_controller::CartesianTask>
{

public:

  typedef std::shared_ptr<Cartesian> Ptr;

  Cartesian(ros::NodeHandle& nh,
            const std::string task_id,
            const XBot::ModelInterface& robot,
            const std::string& distal_link,
            const std::string& base_link,
            const OpenSoT::AffineHelper& qddot,
            const bool use_mesh = true)
    :OpenSoT::tasks::acceleration::Cartesian(task_id,robot,distal_link,base_link,qddot)
    ,TaskRosWrapperInterface<wolf_controller::CartesianTask>(task_id,nh)
    ,_use_mesh(use_mesh)
    ,_distal_link(distal_link)
    ,_base_link(base_link)
    ,_server(wolf_controller::_robot_name+"/wolf_controller/marker/"+_task_id)
  {

    _urdf = robot.getUrdf();

    _control_type = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
    _is_continuous = 1;
    _task_active = -1;
    _position_feedback_active = -1;
    MakeMarker(_distal_link, _base_link, false, _control_type , true);
    //MakeMenu();

    _server.applyChanges();

    // Create the interactive marker
    //marker_server_.reset(new interactive_markers::InteractiveMarkerServer(wolf_controller::_robot_name+"/wolf_controller/marker/"+_task_id));
    //visualization_msgs::InteractiveMarker&& marker = createInteractiveMarker(actual_pose,getBaseLink());
    //marker_server_->insert(marker,boost::bind(&Cartesian::processFeedback, this, _1));
    //marker_server_->applyChanges();
    //
    //// Setup the interpolator
    trj_ = std::make_shared<wolf_controller::CartesianTrajectory>(this);

    std::string topic_name = task_id + "/wp";
    _way_points_pub = nh.advertise<geometry_msgs::PoseArray>(topic_name, 1, true);

  }

  void MakeMarker(const std::string &distal_link, const std::string &base_link,
                  bool fixed, unsigned int interaction_mode, bool show)
  {
      namespace pl = std::placeholders;

      ROS_INFO("Creating marker %s -> %s\n", base_link.c_str(), distal_link.c_str());
      _int_marker.header.frame_id = base_link;
      _int_marker.scale = 0.5;

      _int_marker.name = distal_link;
      _int_marker.description = "";

      // insert STL
      makeSTLControl(_int_marker);
      _int_marker.controls[0].interaction_mode = interaction_mode;
      if(show)
      {
          createInteractiveMarkerControl(1,1,0,0,interaction_mode);
          createInteractiveMarkerControl(1,0,1,0,interaction_mode);
          createInteractiveMarkerControl(1,0,0,1,interaction_mode);
      }
      Eigen::Affine3d _start_pose;
      getActualPose(_start_pose);
      EigenAffine3dToVisualizationPose(_start_pose, _int_marker);
      _server.insert(_int_marker,boost::bind(&Cartesian::processFeedback, this, _1));
  }

  void createInteractiveMarkerControl(const double qw, const double qx, const double qy, const double qz,
                                                       const unsigned int interaction_mode)
  {
      _control.orientation.w = qw;
      _control.orientation.x = qx;
      _control.orientation.y = qy;
      _control.orientation.z = qz;
      if(interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D)
      {
          _control.name = "rotate_x";
          _control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
          _int_marker.controls.push_back(_control);
          _control.name = "move_x";
          _control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
          _int_marker.controls.push_back(_control);
      }
      else if(interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D)
      {
          _control.name = "move_x";
          _control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
          _int_marker.controls.push_back(_control);
      }
      else if(interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D)
      {
          _control.name = "rotate_x";
          _control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
          _int_marker.controls.push_back(_control);
      }
      else throw std::invalid_argument("Invalid interaction mode!");
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
    server_->registerVariable<double>("kp_x",            Kp(0,0), boost::bind(&TaskRosWrapperInterface::setKpX,this,_1)            ,"Kp(0,0)", 0.0, 10000.0);
    server_->registerVariable<double>("kp_y",            Kp(1,1), boost::bind(&TaskRosWrapperInterface::setKpY,this,_1)            ,"Kp(1,1)", 0.0, 10000.0);
    server_->registerVariable<double>("kp_z",            Kp(2,2), boost::bind(&TaskRosWrapperInterface::setKpZ,this,_1)            ,"Kp(2,2)", 0.0, 10000.0);
    server_->registerVariable<double>("kp_roll",         Kp(3,3), boost::bind(&TaskRosWrapperInterface::setKpRoll,this,_1)         ,"Kp(3,3)", 0.0, 10000.0);
    server_->registerVariable<double>("kp_pitch",        Kp(4,4), boost::bind(&TaskRosWrapperInterface::setKpPitch,this,_1)        ,"Kp(4,4)", 0.0, 10000.0);
    server_->registerVariable<double>("kp_yaw",          Kp(5,5), boost::bind(&TaskRosWrapperInterface::setKpYaw,this,_1)          ,"Kp(5,5)", 0.0, 10000.0);
    Eigen::Matrix6d Kd = getKd();
    server_->registerVariable<double>("kd_x",            Kd(0,0), boost::bind(&TaskRosWrapperInterface::setKdX,this,_1)            ,"Kd(0,0)", 0.0, 10000.0);
    server_->registerVariable<double>("kd_y",            Kd(1,1), boost::bind(&TaskRosWrapperInterface::setKdY,this,_1)            ,"Kd(1,1)", 0.0, 10000.0);
    server_->registerVariable<double>("kd_z",            Kd(2,2), boost::bind(&TaskRosWrapperInterface::setKdZ,this,_1)            ,"Kd(2,2)", 0.0, 10000.0);
    server_->registerVariable<double>("kd_roll",         Kd(3,3), boost::bind(&TaskRosWrapperInterface::setKdRoll,this,_1)         ,"Kd(3,3)", 0.0, 10000.0);
    server_->registerVariable<double>("kd_pitch",        Kd(4,4), boost::bind(&TaskRosWrapperInterface::setKdPitch,this,_1)        ,"Kd(4,4)", 0.0, 10000.0);
    server_->registerVariable<double>("kd_yaw",          Kd(5,5), boost::bind(&TaskRosWrapperInterface::setKdYaw,this,_1)          ,"Kd(5,5)", 0.0, 10000.0);
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
    for(unsigned int i=0; i<wolf_controller::_cartesian_names.size(); i++)
    {
      if (!nh_.getParam("gains/"+_task_id+"/Kp/" + wolf_controller::_cartesian_names[i] , Kp(i,i)))
      {
        ROS_WARN("No Kp.%s gain given for task %s in the namespace: %s, using an identity matrix. ",wolf_controller::_cartesian_names[i].c_str(),_task_id.c_str(),nh_.getNamespace().c_str());
        use_identity = true;
      }
      if (!nh_.getParam("gains/"+_task_id+"/Kd/"  + wolf_controller::_cartesian_names[i] , Kd(i,i)))
      {
        ROS_WARN("No Kd.%s gain given for task %s in the namespace: %s, using an identity matrix. ",wolf_controller::_cartesian_names[i].c_str(),_task_id.c_str(),nh_.getNamespace().c_str());
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

    std::string type;
    if (!nh_.getParam("gains/"+_task_id+"/type" , type))
      ROS_WARN("No gains type given for task %s in the namespace: %s, using the default value loaded from the task",_task_id.c_str(),nh_.getNamespace().c_str());
    else
      if(type == "acceleration")
        setGainType(OpenSoT::tasks::acceleration::GainType::Acceleration);
      else if (type == "force")
        setGainType(OpenSoT::tasks::acceleration::GainType::Force);
      else
        throw std::runtime_error("Wrong gain type, possible values are 'acceleration' or 'force'");
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
      wolf_controller::rotToRpy(tmp_affine3d_.linear(),tmp_vector3d_);
      wolf_controller::affine3dToPose(tmp_affine3d_,rt_pub_->msg_.pose_actual);
      wolf_controller::vector6dToTwist(tmp_vector6d_,rt_pub_->msg_.twist_actual);
      wolf_controller::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.rpy_actual);

      // REFERENCE VALUES
      getReference(tmp_affine3d_);
      tmp_vector6d_ = getCachedVelocityReference();
      wolf_controller::rotToRpy(tmp_affine3d_.linear(),tmp_vector3d_);
      wolf_controller::affine3dToPose(tmp_affine3d_,rt_pub_->msg_.pose_reference);
      wolf_controller::vector6dToTwist(tmp_vector6d_,rt_pub_->msg_.twist_reference);
      wolf_controller::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.rpy_reference);

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
      //tmp_vector6d_.setZero();
      //tmp_affine3d_.setIdentity();
      //tmp_affine3d_ = *buffer_pose_reference_.readFromRT();
      // Interpolation
      trj_->update(wolf_controller::_period);
      trj_->getReference(tmp_affine3d_,&tmp_vector6d_);
      setReference(tmp_affine3d_,tmp_vector6d_);
    }
    OpenSoT::tasks::acceleration::Cartesian::_update(x);
  }

  virtual bool reset() override
  {
    bool res = OpenSoT::tasks::acceleration::Cartesian::reset();
    //getActualPose(tmp_affine3d_);
    ////buffer_pose_reference_.writeFromNonRT(tmp_affine3d_);
    //_server.clear();
    //_server.applyChanges();
    //visualization_msgs::InteractiveMarker&& marker = createInteractiveMarker(tmp_affine3d_,getBaseLink());
    //_server.insert(marker,boost::bind(&Cartesian::processFeedback, this, _1));
    //_server.applyChanges();

    MakeMarker(_distal_link, _base_link, false, _control_type , true);
    MakeMenu();
    _server.applyChanges();
    trj_->reset();
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

    wolf_controller::quatToRot(orientation_reference,R);

    pose_reference.translation() = translation_reference;
    pose_reference.linear() = R;

    //buffer_pose_reference_.writeFromNonRT(pose_reference);

    if(_is_continuous == -1)
      trj_->setWayPoint(pose_reference,0.1);
  }

private:

  visualization_msgs::InteractiveMarkerControl& makeSTLControl(visualization_msgs::InteractiveMarker &msg )
  {
      _control2.always_visible = true;

      if(_use_mesh && _urdf.getLink(_distal_link) != NULL)
          _control2.markers.push_back( makeSTL(msg) );
      else
          _control2.markers.push_back( makeSphere(msg) );

      msg.controls.push_back( _control2 );

      return msg.controls.back();
  }

  visualization_msgs::Marker makeSphere( visualization_msgs::InteractiveMarker &msg )
  {
      _marker.type = visualization_msgs::Marker::SPHERE;
      _marker.scale.x = msg.scale * 0.45;//0.45
      _marker.scale.y = msg.scale * 0.45;
      _marker.scale.z = msg.scale * 0.45;
      _marker.color.r = 0.5;
      _marker.color.g = 0.5;
      _marker.color.b = 1.5;
      _marker.color.a = 1.0;

      return _marker;
  }

  KDL::Frame getPose(const std::string& base_link, const std::string& distal_link)
  {

      for(unsigned int i = 0; i < 10; ++i){
          try{
              ros::Time now = ros::Time::now();
              _listener.waitForTransform(base_link, distal_link,ros::Time(0),ros::Duration(1.0));

              _listener.lookupTransform(base_link, distal_link,
                                        ros::Time(0), _transform);
          }
          catch (tf::TransformException ex){
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
          }}

      KDL::Frame transform_KDL;
      tf::TransformTFToKDL(_transform, transform_KDL);

      return transform_KDL;
  }

  template <class Marker_Type>
  inline void KDLFrameToVisualizationPose(const KDL::Frame& Frame, Marker_Type& Marker)
  {
      Marker.pose.position.x = Frame.p.x();
      Marker.pose.position.y = Frame.p.y();
      Marker.pose.position.z = Frame.p.z();
      double qx,qy,qz,qw;
      Frame.M.GetQuaternion(qx,qy,qz,qw);
      Marker.pose.orientation.x = qx;
      Marker.pose.orientation.y = qy;
      Marker.pose.orientation.z = qz;
      Marker.pose.orientation.w = qw;
  }

  template <class Marker_Type>
  inline void EigenAffine3dToVisualizationPose(const Eigen::Affine3d& Frame, Marker_Type& Marker)
  {
      Marker.pose.position.x = Frame.translation().x();
      Marker.pose.position.y = Frame.translation().y();
      Marker.pose.position.z = Frame.translation().z();
      Eigen::Quaterniond q(Frame.linear());
      Marker.pose.orientation.x = q.x();
      Marker.pose.orientation.y = q.y();
      Marker.pose.orientation.z = q.z();
      Marker.pose.orientation.w = q.w();
  }

  void URDFPoseToKDLFrame(const urdf::Pose& Pose, KDL::Frame& Frame)
  {
      Frame.p.x(Pose.position.x);
      Frame.p.y(Pose.position.y);
      Frame.p.z(Pose.position.z);
      Frame.M = Frame.M.Quaternion(Pose.rotation.x, Pose.rotation.y, Pose.rotation.z, Pose.rotation.w);
  }

  visualization_msgs::Marker makeSTL( visualization_msgs::InteractiveMarker &msg )
  {
      auto link = _urdf.getLink(_distal_link);
      auto controlled_link = link;

  #if ROS_VERSION_MINOR <= 12
  #define STATIC_POINTER_CAST boost::static_pointer_cast
  #else
  #define STATIC_POINTER_CAST std::static_pointer_cast
  #endif

      KDL::Frame T; T.Identity();
      while(!link->visual)
      {
          if(!link->parent_joint)
          {
              XBot::Logger::warning("Unable to find mesh for link %s \n", _distal_link.c_str());
              return makeSphere(msg);
          }
          link = _urdf.getLink(link->parent_joint->parent_link_name);
      }

      T = getPose(controlled_link->name, link->name);
      KDL::Frame T_marker;
      URDFPoseToKDLFrame(link->visual->origin, T_marker);
      T = T*T_marker;
      KDLFrameToVisualizationPose(T, _marker);

      //Eigen::Affine3d _start_pose;
      //getActualPose(_start_pose);
      //EigenAffine3dToVisualizationPose(_start_pose, _int_marker);

      _marker.color.r = 0.5;
      _marker.color.g = 0.5;
      _marker.color.b = 0.5;

      if(link->visual->geometry->type == urdf::Geometry::MESH)
      {
          _marker.type = visualization_msgs::Marker::MESH_RESOURCE;

          auto mesh =
                  STATIC_POINTER_CAST<urdf::Mesh>(link->visual->geometry);

          _marker.mesh_resource = mesh->filename;
          _marker.scale.x = mesh->scale.x;
          _marker.scale.y = mesh->scale.y;
          _marker.scale.z = mesh->scale.z;
      }
      else if(link->visual->geometry->type == urdf::Geometry::BOX)
      {
          _marker.type = visualization_msgs::Marker::CUBE;

          auto mesh =
                  STATIC_POINTER_CAST<urdf::Box>(link->visual->geometry);

          //KDL::Frame T_marker;
          //URDFPoseToKDLFrame(link->visual->origin, T_marker);

          _marker.scale.x = mesh->dim.x;
          _marker.scale.y = mesh->dim.y;
          _marker.scale.z = mesh->dim.z;

      }
      else if(link->visual->geometry->type == urdf::Geometry::CYLINDER)
      {
          _marker.type = visualization_msgs::Marker::CYLINDER;

          auto mesh =
                  STATIC_POINTER_CAST<urdf::Cylinder>(link->visual->geometry);

          //KDL::Frame T_marker;
          //URDFPoseToKDLFrame(link->visual->origin, T_marker);

          _marker.scale.x = _marker.scale.y = mesh->radius;
          _marker.scale.z = mesh->length;
      }
      else if(link->visual->geometry->type == urdf::Geometry::SPHERE)
      {
          _marker.type = visualization_msgs::Marker::SPHERE;

          auto mesh =
                  STATIC_POINTER_CAST<urdf::Sphere>(link->visual->geometry);

          KDL::Frame T_marker;
          URDFPoseToKDLFrame(link->visual->origin, T_marker);

          _marker.scale.x = _marker.scale.y = _marker.scale.z = 2.*mesh->radius;
      }

      _marker.color.a = .9;
      return _marker;
  }

  void MakeMenu()
  {

      namespace pl = std::placeholders;

      _menu_entry_counter = 0;

      _continuous_control_entry = _menu_handler.insert("Continuous Ctrl",std::bind(std::mem_fn(&Cartesian::setContinuousCtrl),
                                                                                     this, pl::_1));
      _menu_handler.setCheckState(_continuous_control_entry, interactive_markers::MenuHandler::UNCHECKED);
      _menu_entry_counter++;

      _way_point_entry = _menu_handler.insert("Add WayPoint");
      _menu_handler.setVisible(_way_point_entry, true);
      _menu_entry_counter++;
      _T_entry = _menu_handler.insert(_way_point_entry, "T [sec]");
      _menu_entry_counter++;
      _offset_menu_entry = _menu_entry_counter;
      for ( int i = 0; i < SECS; i++ )
      {
          std::ostringstream s;
          s <<i+1;
          _T_last = _menu_handler.insert( _T_entry, s.str(),
                                          std::bind(std::mem_fn(&Cartesian::wayPointCallBack),
                                                      this, pl::_1));
          _menu_entry_counter++;
          _menu_handler.setCheckState(_T_last, interactive_markers::MenuHandler::UNCHECKED );
      }
      _reset_all_way_points_entry = _menu_handler.insert(_way_point_entry, "Reset All",std::bind(std::mem_fn(&Cartesian::resetAllWayPoints),
                                                                                                  this, pl::_1));
      _menu_entry_counter++;
      _reset_last_way_point_entry = _menu_handler.insert(_way_point_entry, "Reset Last",std::bind(std::mem_fn(&Cartesian::resetLastWayPoints),
                                                                                                    this, pl::_1));
      _menu_entry_counter++;

      _send_way_points_entry = _menu_handler.insert(_way_point_entry, "Send",std::bind(std::mem_fn(&Cartesian::sendWayPoints),
                                                                                         this, pl::_1));
      _menu_entry_counter++;

      _properties_entry = _menu_handler.insert("Properties");
      _menu_entry_counter++;
      _task_is_active_entry = _menu_handler.insert(_properties_entry, "Task Active",std::bind(std::mem_fn(&Cartesian::activateTask),
                                                                                                this, pl::_1));
      _menu_entry_counter++;
      _menu_handler.setCheckState(_task_is_active_entry, interactive_markers::MenuHandler::CHECKED );

      _menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
      _menu_control.always_visible = true;

      _int_marker.controls.push_back(_menu_control);

      _menu_handler.apply(_server, _int_marker.name);
  }

  void sendWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
      if(_waypoints.empty()) return;

      std::partial_sum(_T.begin(), _T.end(), _T.begin());

      wolf_controller::CartesianTrajectory::WayPointVector wpv;

      for(int i = 0; i < _waypoints.size(); i++)
      {
          wolf_controller::CartesianTrajectory::WayPoint wp;

          tf::poseMsgToEigen(_waypoints[i], wp.T_ref);
          wp.duration = _T.at(i);

          wpv.push_back(wp);
      }

      trj_->setWayPoints(wpv);
      publishWP(_waypoints);

      _waypoints.clear();
      _T.clear();
  }

  void activateTask(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
      _task_active *= -1;

      if(_task_active == 1)
      {
          _menu_handler.setCheckState(_task_is_active_entry, interactive_markers::MenuHandler::UNCHECKED);
          setActive(false);
      }
      else if(_task_active == -1)
      {
          _menu_handler.setCheckState(_task_is_active_entry, interactive_markers::MenuHandler::CHECKED);
          setActive(true);
      }

      _menu_handler.reApply(_server);
      _server.applyChanges();
  }


  void resetMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
      if(_task_active)
      {
          _activateTask(false);
      }
      else
      {
          _activateTask(true);
      }

      clearMarker(_req, _res);
      spawnMarker(_req, _res);
  }

  void resetLastWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
      namespace pl = std::placeholders;

      _T.pop_back();
      _waypoints.pop_back();
      clearMarker(_req, _res);

      ROS_INFO("RESET LAST WAYPOINT!");

      if(_waypoints.empty())
          spawnMarker(_req, _res);
      else{
          if(_server.empty())
          {
              tf::poseMsgToKDL(_waypoints.back(),_start_pose);
              //tf::PoseMsgToKDL(_waypoints.back(),_start_pose);
              _actual_pose = _start_pose;

              KDLFrameToVisualizationPose(_start_pose, _int_marker);

              _server.insert(_int_marker, std::bind(std::mem_fn(&Cartesian::processFeedback),this, pl::_1));
              _menu_handler.apply(_server, _int_marker.name);
              _server.applyChanges();
          }
      }
      publishWP(_waypoints);
  }

  void resetAllWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
      _T.clear();
      _waypoints.clear();
      resetMarker(feedback);
      publishWP(_waypoints);
      ROS_INFO("RESETTING ALL WAYPOINTS!");
  }

  void wayPointCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
      //In base_link
      tf::poseMsgToKDL(feedback->pose, _actual_pose);
      double qx,qy,qz,qw;
      _actual_pose.M.GetQuaternion(qx,qy,qz,qw);

      double T = double(feedback->menu_entry_id-offset_menu_entry);


      if(_is_continuous == 1)
      {
          ROS_INFO("\n %s set waypoint @: \n pos = [%f, %f, %f],\n orient = [%f, %f, %f, %f],\n of %.1f secs",
                   _int_marker.name.c_str(),
                   _actual_pose.p.x(), _actual_pose.p.y(), _actual_pose.p.z(),
                   qx,qy,qz,qw, T);

          _waypoints.push_back(feedback->pose);
          _T.push_back(T);

          publishWP(_waypoints);

      }
  }

  void publishWP(const std::vector<geometry_msgs::Pose>& wps)
  {
      geometry_msgs::PoseArray msg;
      for(unsigned int i = 0; i < wps.size(); ++i)
          msg.poses.push_back(wps[i]);

      msg.header.frame_id = _base_link;
      msg.header.stamp = ros::Time::now();

      _way_points_pub.publish(msg);
  }

  bool setTrj(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
      clearMarker(req, res);
      spawnMarker(req, res);
      return true;
  }

  bool setContinuous(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
      clearMarker(req, res);
      spawnMarker(req,res);
      return true;
  }

  void _activateTask(const bool is_active)
  {
      if(is_active)
      {
          _task_active = -1;
          _menu_handler.setCheckState(_task_is_active_entry,
                                      interactive_markers::MenuHandler::CHECKED);
      }
      else
      {
          _task_active = 1;
          _menu_handler.setCheckState(_task_is_active_entry,
                                      interactive_markers::MenuHandler::UNCHECKED);
      }

      _menu_handler.reApply(_server);
      _server.applyChanges();
  }

  void _activatePositionFeedBack(const bool is_active)
  {
      if(is_active){
          _position_feedback_active = -1;
          _menu_handler.setCheckState(_position_feedback_is_active_entry, interactive_markers::MenuHandler::CHECKED);}
      else{
          _position_feedback_active = 1;
          _menu_handler.setCheckState(_position_feedback_is_active_entry, interactive_markers::MenuHandler::UNCHECKED);}

      _menu_handler.reApply(_server);
      _server.applyChanges();
  }

  bool clearMarker(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
      if(!_server.empty())
      {
          _server.erase(_int_marker.name);
          _server.applyChanges();
      }
      return true;
  }

  bool spawnMarker(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
      namespace pl = std::placeholders;

      if(_server.empty())
      {
        Eigen::Affine3d _start_pose;
        getActualPose(_start_pose);
        EigenAffine3dToVisualizationPose(_start_pose, _int_marker);
        _server.insert(_int_marker,boost::bind(&Cartesian::processFeedback, this, _1));
        _server.applyChanges();
      }
      return true;
  }

  void setContinuousCtrl(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
      _is_continuous *= -1;

      if(_is_continuous == 1)
      {
          _menu_handler.setCheckState(_continuous_control_entry, interactive_markers::MenuHandler::UNCHECKED);
          _menu_handler.setVisible(_way_point_entry, true);
          _waypoints.clear();
          _T.clear();
          setContinuous(_req,_res);
      }
      else if(_is_continuous == -1)
      {
          _menu_handler.setCheckState(_continuous_control_entry, interactive_markers::MenuHandler::CHECKED);
          _menu_handler.setVisible(_way_point_entry, false);
          _waypoints.clear();
          _T.clear();
          setTrj(_req, _res);
      }

      _menu_handler.reApply(_server);
      _server.applyChanges();
  }

  ros::Publisher _way_points_pub;


  /**
   * @brief _waypoints contains all the waypoints BUT not the initial position!
   */
  std::vector<geometry_msgs::Pose> _waypoints;

  /**
   * @brief _T contains the times of each waypoint-trajectory
   */
  std::vector<float> _T;

  /**
   * @brief control
   */
  visualization_msgs::InteractiveMarkerControl _control;

  /**
   * @brief _control2
   */
  visualization_msgs::InteractiveMarkerControl _control2;

  int _menu_entry_counter;
  int _offset_menu_entry;
  int _control_type;
  int _is_continuous;
  int offset_menu_entry;
  int _task_active;
  int _position_feedback_active;

  std_srvs::EmptyRequest _req;
  std_srvs::EmptyResponse _res;

  visualization_msgs::InteractiveMarker _int_marker;
  /**
   * @brief _server interactive marker server used by the marker
   */
  interactive_markers::InteractiveMarkerServer _server;
  visualization_msgs::Marker _marker;

  interactive_markers::MenuHandler _menu_handler;
  interactive_markers::MenuHandler::EntryHandle _reset_marker_entry;
  interactive_markers::MenuHandler::EntryHandle _way_point_entry;
  interactive_markers::MenuHandler::EntryHandle _T_entry;
  interactive_markers::MenuHandler::EntryHandle _T_last;
  interactive_markers::MenuHandler::EntryHandle _reset_last_way_point_entry;
  interactive_markers::MenuHandler::EntryHandle _reset_all_way_points_entry;
  interactive_markers::MenuHandler::EntryHandle _send_way_points_entry;
  interactive_markers::MenuHandler::EntryHandle _global_control_entry;
  interactive_markers::MenuHandler::EntryHandle _continuous_control_entry;
  interactive_markers::MenuHandler::EntryHandle _properties_entry;
  interactive_markers::MenuHandler::EntryHandle _task_is_active_entry;
  interactive_markers::MenuHandler::EntryHandle _position_feedback_is_active_entry;
  interactive_markers::MenuHandler::EntryHandle _base_link_entry;
  std::vector<interactive_markers::MenuHandler::EntryHandle> _link_entries;
  visualization_msgs::InteractiveMarkerControl  _menu_control;

  tf::TransformListener _listener;
  tf::StampedTransform _transform;

  /**
   * @brief _urdf model description of the robot
   */
  urdf::ModelInterface _urdf;

  bool _use_mesh;

  /**
   * @brief _base_link used by the marker
   */
  std::string _base_link;
  /**
   * @brief _distal_link used by the marker
   */
  std::string _distal_link;

  /**
   * @brief _start_pose is the starting pose of the marker, taken from the current pose of the robot
   */
  KDL::Frame _start_pose;
  KDL::Frame _actual_pose;


};

#endif // ROS_WRAPPERS_CARTESIAN_H

