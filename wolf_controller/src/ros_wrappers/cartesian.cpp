/**
 * @file cartesian.cpp
 * @author Gennaro Raiola
 * @date 24 April, 2023
 * @brief This file contains the cartesian task wrapper for ROS
 */

// WoLF
#include <wolf_controller/ros_wrappers/cartesian.h>

Cartesian::Cartesian(ros::NodeHandle& nh,
                     const std::string task_id,
                     const XBot::ModelInterface& robot,
                     const std::string& distal_link,
                     const std::string& base_link,
                     const OpenSoT::AffineHelper& qddot,
                     const bool use_mesh)
  :OpenSoT::tasks::acceleration::Cartesian(task_id,robot,distal_link,base_link,qddot)
  ,TaskRosWrapperInterface<wolf_msgs::CartesianTask>(task_id,nh)
  ,use_mesh_(use_mesh)
  ,interactive_marker_server_(wolf_controller::_robot_name+"/wolf_controller/marker/"+_task_id)
{

  // Get the urdf (used for the mesh)
  urdf_ = robot.getUrdf();

  // Get the urdf links (used for the base frame selection)
  urdf_.getLinks(links_);

  // Create the marker
  control_type_ = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  is_continuous_ = false;
  makeMarker(getDistalLink(),getBaseLink(),control_type_,true);
  makeMenu();
  interactive_marker_server_.applyChanges();

  // Setup the interpolator
  trj_ = std::make_shared<wolf_controller::CartesianTrajectory>(this);

  // Create the waypoints publisher
  waypoints_pub_ = nh.advertise<geometry_msgs::PoseArray>(task_id + "/wp", 1, true);

  // Create the reference subscriber
  reference_sub_ = nh.subscribe("reference/"+_task_id, 1000, &Cartesian::referenceCallback, this);
}

void Cartesian::registerReconfigurableVariables()
{
  double lambda1 = getLambda();
  double lambda2 = getLambda2();
  double weight  = getWeight()(0,0);
  ddr_server_->registerVariable<double>("set_lambda_1",    lambda1,     boost::bind(&TaskRosWrapperInterface::setLambda1,this,_1)    ,"set lambda 1"   ,0.0,1000.0);
  ddr_server_->registerVariable<double>("set_lambda_2",    lambda2,     boost::bind(&TaskRosWrapperInterface::setLambda2,this,_1)    ,"set lambda 2"   ,0.0,1000.0);
  ddr_server_->registerVariable<double>("set_weight_diag", weight,      boost::bind(&TaskRosWrapperInterface::setWeightDiag,this,_1) ,"set weight diag",0.0,1000.0);
  Eigen::Matrix6d Kp = getKp();
  ddr_server_->registerVariable<double>("kp_x",            Kp(0,0), boost::bind(&TaskRosWrapperInterface::setKpX,this,_1)            ,"Kp(0,0)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kp_y",            Kp(1,1), boost::bind(&TaskRosWrapperInterface::setKpY,this,_1)            ,"Kp(1,1)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kp_z",            Kp(2,2), boost::bind(&TaskRosWrapperInterface::setKpZ,this,_1)            ,"Kp(2,2)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kp_roll",         Kp(3,3), boost::bind(&TaskRosWrapperInterface::setKpRoll,this,_1)         ,"Kp(3,3)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kp_pitch",        Kp(4,4), boost::bind(&TaskRosWrapperInterface::setKpPitch,this,_1)        ,"Kp(4,4)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kp_yaw",          Kp(5,5), boost::bind(&TaskRosWrapperInterface::setKpYaw,this,_1)          ,"Kp(5,5)", 0.0, 10000.0);
  Eigen::Matrix6d Kd = getKd();
  ddr_server_->registerVariable<double>("kd_x",            Kd(0,0), boost::bind(&TaskRosWrapperInterface::setKdX,this,_1)            ,"Kd(0,0)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kd_y",            Kd(1,1), boost::bind(&TaskRosWrapperInterface::setKdY,this,_1)            ,"Kd(1,1)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kd_z",            Kd(2,2), boost::bind(&TaskRosWrapperInterface::setKdZ,this,_1)            ,"Kd(2,2)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kd_roll",         Kd(3,3), boost::bind(&TaskRosWrapperInterface::setKdRoll,this,_1)         ,"Kd(3,3)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kd_pitch",        Kd(4,4), boost::bind(&TaskRosWrapperInterface::setKdPitch,this,_1)        ,"Kd(4,4)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kd_yaw",          Kd(5,5), boost::bind(&TaskRosWrapperInterface::setKdYaw,this,_1)          ,"Kd(5,5)", 0.0, 10000.0);
  ddr_server_->publishServicesTopics();
}

void Cartesian::loadParams()
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

void Cartesian::updateCost(const Eigen::VectorXd& x)
{
  cost_ = computeCost(x);
}

void Cartesian::publish(const ros::Time& time)
{
  if(rt_pub_->trylock())
  {
    rt_pub_->msg_.header.frame_id = getBaseLink();
    rt_pub_->msg_.header.stamp = time;

    // ACTUAL VALUES
    getActualPose(tmp_affine3d_);
    getActualTwist(tmp_vector6d_);
    wolf_controller_utils::rotToRpy(tmp_affine3d_.linear(),tmp_vector3d_);
    wolf_controller_utils::affine3dToPose(tmp_affine3d_,rt_pub_->msg_.pose_actual);
    wolf_controller_utils::vector6dToTwist(tmp_vector6d_,rt_pub_->msg_.twist_actual);
    wolf_controller_utils::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.rpy_actual);

    // REFERENCE VALUES
    getReference(tmp_affine3d_);
    tmp_vector6d_ = getCachedVelocityReference();
    wolf_controller_utils::rotToRpy(tmp_affine3d_.linear(),tmp_vector3d_);
    wolf_controller_utils::affine3dToPose(tmp_affine3d_,rt_pub_->msg_.pose_reference);
    wolf_controller_utils::vector6dToTwist(tmp_vector6d_,rt_pub_->msg_.twist_reference);
    wolf_controller_utils::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.rpy_reference);

    // COST
    rt_pub_->msg_.cost = cost_;

    // PUBLISH
    rt_pub_->unlockAndPublish();
  }
}

void Cartesian::_update(const Eigen::VectorXd& x)
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
    // Interpolation
    trj_->update(wolf_controller::_period);
    trj_->getReference(tmp_affine3d_,&tmp_vector6d_);
    setReference(tmp_affine3d_,tmp_vector6d_);
  }
  OpenSoT::tasks::acceleration::Cartesian::_update(x);
}

bool Cartesian::reset()
{
  bool res = OpenSoT::tasks::acceleration::Cartesian::reset(); // Task's reset
  makeMarker(getDistalLink(),getBaseLink(),control_type_,true);
  menu_handler_.reApply(interactive_marker_server_);
  interactive_marker_server_.applyChanges();
  waypoints_.clear();
  publishWP(waypoints_);
  trj_->reset();
  return res;
}

void Cartesian::makeMarker(const std::string &distal_link, const std::string &base_link, unsigned int interaction_mode, bool show)
{
  ROS_DEBUG("Creating marker %s -> %s\n", base_link.c_str(), distal_link.c_str());

  interactive_marker_.header.frame_id = base_link;
  interactive_marker_.scale = 0.5;

  interactive_marker_.name = distal_link;
  interactive_marker_.description = "";

  // Insert STL
  makeSTLControl(interactive_marker_);
  interactive_marker_.controls[0].interaction_mode = interaction_mode;
  if(show)
  {
    createInteractiveMarkerControl(1,1,0,0,interaction_mode);
    createInteractiveMarkerControl(1,0,1,0,interaction_mode);
    createInteractiveMarkerControl(1,0,0,1,interaction_mode);
  }
  Eigen::Affine3d start_pose;
  getActualPose(start_pose);
  EigenAffine3dToVisualizationPose(start_pose, interactive_marker_);
  interactive_marker_server_.insert(interactive_marker_,boost::bind(&Cartesian::processFeedback, this, _1));
}

void Cartesian::createInteractiveMarkerControl(const double qw, const double qx, const double qy, const double qz,
                                               const unsigned int interaction_mode)
{
  visualization_msgs::InteractiveMarkerControl tmp_control;
  tmp_control.orientation.w = qw;
  tmp_control.orientation.x = qx;
  tmp_control.orientation.y = qy;
  tmp_control.orientation.z = qz;
  if(interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D)
  {
    tmp_control.name = "rotate_x";
    tmp_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker_.controls.push_back(tmp_control);
    tmp_control.name = "move_x";
    tmp_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker_.controls.push_back(tmp_control);
  }
  else if(interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D)
  {
    tmp_control.name = "move_x";
    tmp_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker_.controls.push_back(tmp_control);
  }
  else if(interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D)
  {
    tmp_control.name = "rotate_x";
    tmp_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker_.controls.push_back(tmp_control);
  }
  else throw std::invalid_argument("Invalid interaction mode!");
}

void Cartesian::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_DEBUG_STREAM( feedback->marker_name << " is now at "
                    << feedback->pose.position.x << ", " << feedback->pose.position.y
                    << ", " << feedback->pose.position.z << " frame " << feedback->header.frame_id );

  Eigen::Affine3d pose_reference = Eigen::Affine3d::Identity();
  tf::poseMsgToEigen(feedback->pose,pose_reference);

  if(is_continuous_ == true)
    trj_->setWayPoint(pose_reference,0.1);
}

void Cartesian::referenceCallback(const wolf_msgs::Cartesian::ConstPtr& msg)
{
  double period = wolf_controller::_period;

  if(last_time_ != 0.0)
    period = msg->header.stamp.toSec() - last_time_;

  Eigen::Affine3d pose_reference = Eigen::Affine3d::Identity();
  tf::poseMsgToEigen(msg->pose,pose_reference);

  // TODO change reference frame if needed

  trj_->setWayPoint(pose_reference,period);

  last_time_ = msg->header.stamp.toSec();
}

visualization_msgs::InteractiveMarkerControl& Cartesian::makeSTLControl(visualization_msgs::InteractiveMarker& msg)
{
  visualization_msgs::InteractiveMarkerControl tmp_control;
  tmp_control.always_visible = true;

  if(use_mesh_ && urdf_.getLink(getDistalLink()) != nullptr)
    tmp_control.markers.push_back(makeSTL(msg));
  else
    tmp_control.markers.push_back(makeSphere(msg));

  msg.controls.push_back(tmp_control);

  return msg.controls.back();
}

visualization_msgs::Marker Cartesian::makeSphere(visualization_msgs::InteractiveMarker& msg)
{
  marker_.type = visualization_msgs::Marker::SPHERE;
  marker_.scale.x = msg.scale * 0.45;
  marker_.scale.y = msg.scale * 0.45;
  marker_.scale.z = msg.scale * 0.45;
  marker_.color.r = 0.5;
  marker_.color.g = 0.5;
  marker_.color.b = 1.5;
  marker_.color.a = 1.0;
  return marker_;
}

Eigen::Affine3d Cartesian::getPose(const std::string& base_link, const std::string& distal_link)
{
  tf::StampedTransform transform;
  for(unsigned int i = 0; i < 10; ++i)
  {
    try
    {
      ros::Time now = ros::Time::now();
      listener_.waitForTransform(base_link, distal_link,ros::Time(0),ros::Duration(1.0));
      listener_.lookupTransform(base_link, distal_link,ros::Time(0),transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  Eigen::Affine3d pose;
  tf::transformTFToEigen(transform,pose);
  return pose;
}



visualization_msgs::Marker Cartesian::makeSTL( visualization_msgs::InteractiveMarker &msg )
{
  auto distal_link = getDistalLink();
  auto link = urdf_.getLink(distal_link);
  auto controlled_link = link;

  while(!link->visual)
  {
    if(!link->parent_joint)
    {
      ROS_WARN_STREAM("Unable to find mesh for link " << distal_link.c_str());
      return makeSphere(msg);
    }
    link = urdf_.getLink(link->parent_joint->parent_link_name);
  }

  Eigen::Affine3d&& actual_pose = getPose(controlled_link->name, link->name);
  EigenAffine3dToVisualizationPose(actual_pose,marker_);

  marker_.color.r = 0.5;
  marker_.color.g = 0.5;
  marker_.color.b = 0.5;

  if(link->visual->geometry->type == urdf::Geometry::MESH)
  {
    marker_.type = visualization_msgs::Marker::MESH_RESOURCE;

    auto mesh = std::static_pointer_cast<urdf::Mesh>(link->visual->geometry);

    marker_.mesh_resource = mesh->filename;
    marker_.scale.x = mesh->scale.x;
    marker_.scale.y = mesh->scale.y;
    marker_.scale.z = mesh->scale.z;
  }
  else if(link->visual->geometry->type == urdf::Geometry::BOX)
  {
    marker_.type = visualization_msgs::Marker::CUBE;

    auto mesh = std::static_pointer_cast<urdf::Box>(link->visual->geometry);

    marker_.scale.x = mesh->dim.x;
    marker_.scale.y = mesh->dim.y;
    marker_.scale.z = mesh->dim.z;

  }
  else if(link->visual->geometry->type == urdf::Geometry::CYLINDER)
  {
    marker_.type = visualization_msgs::Marker::CYLINDER;

    auto mesh = std::static_pointer_cast<urdf::Cylinder>(link->visual->geometry);

    marker_.scale.x = marker_.scale.y = mesh->radius;
    marker_.scale.z = mesh->length;
  }
  else if(link->visual->geometry->type == urdf::Geometry::SPHERE)
  {
    marker_.type = visualization_msgs::Marker::SPHERE;

    auto mesh = std::static_pointer_cast<urdf::Sphere>(link->visual->geometry);

    marker_.scale.x = marker_.scale.y = marker_.scale.z = 2.*mesh->radius;
  }

  marker_.color.a = .9;
  return marker_;
}

void Cartesian::makeMenu()
{

  continuous_control_entry_ = menu_handler_.insert("Continuous Ctrl",boost::bind(&Cartesian::setContinuousCtrl,this,_1));
  menu_handler_.setCheckState(continuous_control_entry_, interactive_markers::MenuHandler::UNCHECKED);

  way_point_entry_ = menu_handler_.insert("Add WayPoint");
  menu_handler_.setVisible(way_point_entry_, true);

  T_entry_ = menu_handler_.insert(way_point_entry_, "T [sec]");
  for (unsigned int i = 0; i < 10; i++ )
  {
    std::ostringstream sec_opt;
    double sec = (i+1) * 0.5; // Make increments of 0.5s
    sec_opt << sec;
    T_last_ = menu_handler_.insert(T_entry_, sec_opt.str(),boost::bind(&Cartesian::wayPointCallBack,this,_1,sec));
    menu_handler_.setCheckState(T_last_,interactive_markers::MenuHandler::UNCHECKED);
  }
  reset_all_way_points_entry_ = menu_handler_.insert(way_point_entry_, "Reset All",boost::bind(&Cartesian::resetAllWayPoints,this,_1));
  reset_lastway_point_entry_ = menu_handler_.insert(way_point_entry_, "Reset Last",boost::bind(&Cartesian::resetLastWayPoints,this,_1));
  send_way_points_entry_ = menu_handler_.insert(way_point_entry_, "Send",boost::bind(&Cartesian::sendWayPoints,this,_1));

  base_link_entry_ = menu_handler_.insert("Base Link");
  menu_handler_.setVisible(base_link_entry_, true);

  std::string distal_link = getDistalLink();
  std::string base_link = getBaseLink();
  for(unsigned int i = 0; i < links_.size(); ++i)
  {
    if(distal_link != links_.at(i)->name)
    {
      interactive_markers::MenuHandler::EntryHandle link_entry =
          menu_handler_.insert(base_link_entry_,links_.at(i)->name,boost::bind(&Cartesian::changeBaseLink,this,_1,links_.at(i)->name));


      if(base_link.compare("world") == 0 && (links_.at(i)->name).compare("world") == 0)
      {
        menu_handler_.setCheckState(link_entry, interactive_markers::MenuHandler::CHECKED );
      }
      else if(base_link.compare(links_.at(i)->name) == 0)
      {
        menu_handler_.setCheckState(link_entry, interactive_markers::MenuHandler::CHECKED );
      }
      else
        menu_handler_.setCheckState(link_entry, interactive_markers::MenuHandler::UNCHECKED );

      map_link_entries_[links_.at(i)->name] = link_entry;
    }
  }

  menu_control_.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  menu_control_.always_visible = true;

  interactive_marker_.controls.push_back(menu_control_);
  menu_handler_.apply(interactive_marker_server_, interactive_marker_.name);
}

void Cartesian::changeBaseLink(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::string new_base_link)
{
  int entry_id = feedback->menu_entry_id;

  menu_handler_.getTitle(entry_id, new_base_link);

  if(new_base_link.compare(getBaseLink()) == 0)
    return;

  if(setBaseLink(new_base_link))
  {
    update(Eigen::VectorXd(1));
    reset();
    for (auto& tmp_map : map_link_entries_)
    {
      if(tmp_map.first == new_base_link)
        menu_handler_.setCheckState(tmp_map.second, interactive_markers::MenuHandler::CHECKED );
      else
        menu_handler_.setCheckState(tmp_map.second, interactive_markers::MenuHandler::UNCHECKED );
    }
    menu_handler_.reApply(interactive_marker_server_);
    interactive_marker_server_.applyChanges();
  }
}

void Cartesian::sendWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if(waypoints_.empty()) return;

  std::partial_sum(T_.begin(),T_.end(),T_.begin());

  wolf_controller::CartesianTrajectory::WayPointVector wpv;

  for(int i = 0; i < waypoints_.size(); i++)
  {
    wolf_controller::CartesianTrajectory::WayPoint wp;

    tf::poseMsgToEigen(waypoints_[i], wp.T_ref);
    wp.duration = T_.at(i);

    wpv.push_back(wp);
  }

  trj_->setWayPoints(wpv);
  publishWP(waypoints_);

  waypoints_.clear();
  T_.clear();
}

void Cartesian::resetMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  clearMarker(req_, res_);
  spawnMarker(req_, res_);
}

void Cartesian::resetLastWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  ROS_INFO("RESET LAST WAYPOINT!");

  if(!T_.empty())
    T_.pop_back();
  if(!waypoints_.empty())
    waypoints_.pop_back();

  clearMarker(req_, res_);

  if(waypoints_.empty())
    spawnMarker(req_, res_);
  else
  {
    if(interactive_marker_server_.empty())
    {
      PoseToVisualizationPose(waypoints_.back(),interactive_marker_);
      interactive_marker_server_.insert(interactive_marker_,boost::bind(&Cartesian::processFeedback,this,_1));
      menu_handler_.reApply(interactive_marker_server_);
      interactive_marker_server_.applyChanges();
    }
  }
  publishWP(waypoints_);
}

void Cartesian::resetAllWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  T_.clear();
  waypoints_.clear();
  resetMarker(feedback);
  publishWP(waypoints_);
  ROS_INFO("RESETTING ALL WAYPOINTS!");
}

void Cartesian::wayPointCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, double T)
{
  double x,y,z,qx,qy,qz,qw;
  x  = feedback->pose.position.x;
  y  = feedback->pose.position.y;
  z  = feedback->pose.position.z;
  qx = feedback->pose.orientation.x;
  qy = feedback->pose.orientation.y;
  qz = feedback->pose.orientation.z;
  qw = feedback->pose.orientation.w;
  if(is_continuous_ == false)
  {
    ROS_INFO("\n %s set waypoint wrt %s @: \n pos = [%f, %f, %f],\n orient = [%f, %f, %f, %f],\n of %.1f secs",
             interactive_marker_.name.c_str(),
             getBaseLink().c_str(),
             x,y,z,
             qx,qy,qz,qw, T);

    waypoints_.push_back(feedback->pose);
    T_.push_back(T);
    publishWP(waypoints_);
  }
}

void Cartesian::publishWP(const std::vector<geometry_msgs::Pose>& wps)
{
  geometry_msgs::PoseArray msg;
  for(unsigned int i = 0; i < wps.size(); ++i)
    msg.poses.push_back(wps[i]);

  msg.header.frame_id = getBaseLink();
  msg.header.stamp = ros::Time::now();

  waypoints_pub_.publish(msg);
}

bool Cartesian::setTrj(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  clearMarker(req, res);
  spawnMarker(req, res);
  return true;
}

bool Cartesian::setContinuous(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  clearMarker(req, res);
  spawnMarker(req,res);
  return true;
}

bool Cartesian::clearMarker(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  if(!interactive_marker_server_.empty())
  {
    interactive_marker_server_.erase(interactive_marker_.name);
    interactive_marker_server_.applyChanges();
  }
  return true;
}

bool Cartesian::spawnMarker(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  if(interactive_marker_server_.empty())
  {
    Eigen::Affine3d start_pose;
    getActualPose(start_pose);
    EigenAffine3dToVisualizationPose(start_pose,interactive_marker_);
    interactive_marker_server_.insert(interactive_marker_,boost::bind(&Cartesian::processFeedback, this, _1));
    menu_handler_.reApply(interactive_marker_server_);
    interactive_marker_server_.applyChanges();
  }
  return true;
}

void Cartesian::setContinuousCtrl(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  is_continuous_ = !is_continuous_;

  if(is_continuous_ == false)
  {
    menu_handler_.setCheckState(continuous_control_entry_, interactive_markers::MenuHandler::UNCHECKED);
    menu_handler_.setVisible(way_point_entry_, true);
    waypoints_.clear();
    T_.clear();
    setContinuous(req_,res_);
  }
  else if(is_continuous_ == true)
  {
    menu_handler_.setCheckState(continuous_control_entry_, interactive_markers::MenuHandler::CHECKED);
    menu_handler_.setVisible(way_point_entry_, false);
    waypoints_.clear();
    T_.clear();
    setTrj(req_, res_);
  }

  menu_handler_.reApply(interactive_marker_server_);
  interactive_marker_server_.applyChanges();
}
