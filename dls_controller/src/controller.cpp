/**
 * @file controller.cpp
 * @author Gennaro Raiola
 * @date 12 June, 2018
 * @brief DLS Controller.
 *
 * This file contains the constructor, destructor, init, stopping and other facilities for the
 * DLS Controller.
 * @see git@gitlab.advr.iit.it:dls-lab/dls_core.git
 */

#include <dls_controller/controller.h>

using namespace XBot;
using namespace Cartesian;

namespace dls_controller {

Controller::Controller()
    :solver_started_(false)
{
}

Controller::~Controller()
{
    if(realtime_pub_)
        delete realtime_pub_;
}

bool Controller::init(hardware_interface::JointCommandAdvInterface* hw,
                      ros::NodeHandle& root_nh,
                      ros::NodeHandle& controller_nh)
{
    // getting the names of the joints from the ROS parameter server
    ROS_DEBUG("Initialize DLS Controller");

    assert(hw);

    //hardware_interface::JointCommandAdvInterface* jt_hw = robot_hw->get<hardware_interface::JointCommandAdvInterface>();

    if (!controller_nh.getParam("joints", joint_names_))
    {
        ROS_ERROR("No joints given in the namespace: %s.", controller_nh.getNamespace().c_str());
        return false;
    }

    // Setting up handles:
    for (unsigned int i = 0; i < joint_names_.size(); i++)
    {
        // Getting joint state handle
        try
        {
            ROS_DEBUG_STREAM("Found joint: "<<joint_names_[i]);
            joint_states_.push_back(hw->getHandle(joint_names_[i]));
        }
        catch(...)
        {
            ROS_ERROR("Error loading joint_states_");
            return false;
        }
    }

    assert(joint_states_.size()>0);

    desired_joint_p_gain_.resize(joint_states_.size());
    desired_joint_i_gain_.resize(joint_states_.size());
    desired_joint_d_gain_.resize(joint_states_.size());
    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {
        // Getting PID gains
        if (!controller_nh.getParam("gains/" + joint_names_[i] + "/p", desired_joint_p_gain_[i]))
        {
            ROS_ERROR("No P gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
            return false;
        }
        if (!controller_nh.getParam("gains/" + joint_names_[i] + "/i", desired_joint_i_gain_[i]))
        {
            ROS_ERROR("No D gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
            return false;
        }
        if (!controller_nh.getParam("gains/" + joint_names_[i] + "/d", desired_joint_d_gain_[i]))
        {
            ROS_ERROR("No I gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
            return false;
        }
        // Check if the values are positive
        if(desired_joint_p_gain_[i]<0.0 || desired_joint_i_gain_[i]<0.0 || desired_joint_d_gain_[i]<0.0)
        {
            ROS_ERROR("PID gains must be positive!");
            return false;
        }
        ROS_DEBUG("P value for joint %i is: %d",i,desired_joint_p_gain_[i]);
        ROS_DEBUG("I value for joint %i is: %d",i,desired_joint_i_gain_[i]);
        ROS_DEBUG("D value for joint %i is: %d",i,desired_joint_d_gain_[i]);
    }

    // Create the ModelInterface from XBot
    XBot::ConfigOptions opt;
    std::string urdf, srdf, problem;
    if(!controller_nh.getParam("robot_description",urdf))
    {
        ROS_ERROR("No robot_description given");
        return false;
    }
    if(!controller_nh.getParam("robot_description_semantic",srdf))
    {
        ROS_ERROR("No robot_description_semantic given");
        return false;
    }
    if(!opt.set_urdf_path("/home/graiola/ros_ws/src/hyq-distro/dls_controller/robots/hyqreal/hyqreal.urdf"))
    {
        ROS_ERROR("Unable to load urdf path");
        return false;
    }
    if(!opt.set_srdf_path("/home/graiola/ros_ws/src/hyq-distro/dls_controller/robots/hyqreal/hyqreal.srdf"))
    {
        ROS_ERROR("Unable to load srdf path");
        return false;
    }
    /*if(!opt.set_urdf(urdf))
   {
       ROS_ERROR("Unable to load urdf");
       return false;
   }*/
    /*if(!opt.set_srdf(srdf))
   {
       ROS_ERROR("Unable to load srdf");
       return false;
   }*/
    if(!opt.generate_jidmap())
    {
        ROS_ERROR("Unable to load jidmap");
        return false;
    }
    opt.set_parameter("is_model_floating_base", true);
    opt.set_parameter<std::string>("model_type", "RBDL");
    xbot_model_ = XBot::ModelInterface::getModel(opt);

    // Set home position defined in the srdf
    Eigen::VectorXd qhome;
    xbot_model_->getRobotState("home", qhome);
    xbot_model_->setJointPosition(qhome);

    // Create the kinematics problem
    //YAML::Node config = YAML::LoadFile("PATH TO FILE"); // FIXME use the ros param server
    if(!controller_nh.getParam("problem_description",problem))
    {
        ROS_ERROR("No problem_description given");
        return false;
    }
    YAML::Node config = YAML::Load(problem);
    ProblemDescription ik_problem(config, xbot_model_);

    // Create the CartesianInterfaceImpl from XBot::Cartesian
    //ci_.reset(new XBot::Cartesian::CartesianInterfaceImpl(xbot_model_,ik_problem));
    std::string impl_name = "OpenSot";
    std::string path_to_shared_lib = XBot::Utils::FindLib("libCartesian" + impl_name + ".so", "LD_LIBRARY_PATH");
    if (path_to_shared_lib == "")
    {
        ROS_ERROR_STREAM("libCartesian" + impl_name + ".so must be listed inside LD_LIBRARY_PATH");
        return false;
    }

    ci_ = SoLib::getFactoryWithArgs<CartesianInterfaceImpl>(path_to_shared_lib,
                                                            impl_name + "Impl",
                                                            xbot_model_, ik_problem);
    ci_->enableOtg(0.004); // FIXME Load correct loop period
    ci_->update(0,0);

    // Resize the variables
    joint_positions_.resize(joint_states_.size()+6);
    joint_velocities_.resize(joint_states_.size()+6);
    joint_accellerations_.resize(joint_states_.size()+6);
    joint_efforts_.resize(joint_states_.size()+6);
    des_joint_positions_.resize(joint_states_.size()+6);
    des_joint_velocities_.resize(joint_states_.size()+6);

    joint_positions_.fill(0.0);
    joint_velocities_.fill(0.0);
    joint_accellerations_.fill(0.0);
    joint_efforts_.fill(0.0);
    des_joint_positions_.fill(0.0);
    des_joint_velocities_.fill(0.0);

    // Create the realtime publishers
    realtime_pub_ = new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "ci/joint_states", 4);
    realtime_pub_->msg_.name.resize(joint_states_.size()+6);
    realtime_pub_->msg_.position.resize(joint_states_.size()+6);
    realtime_pub_->msg_.velocity.resize(joint_states_.size()+6);
    realtime_pub_->msg_.effort.resize(joint_states_.size()+6);
    realtime_pub_->msg_.name[0] = "x"; //FIXME
    realtime_pub_->msg_.name[1] = "y";
    realtime_pub_->msg_.name[2] = "z";
    realtime_pub_->msg_.name[3] = "r";
    realtime_pub_->msg_.name[4] = "p";
    realtime_pub_->msg_.name[5] = "y";
    for (unsigned int i = 0; i < joint_names_.size(); i++)
        realtime_pub_->msg_.name[i+6] = joint_names_[i];

    // Reference for the com
    com_ref_sub_ = controller_nh.subscribe("com_ref", 1, &Controller::setComReference, this);
    // Position of the com
    com_pub_ = controller_nh.advertise<geometry_msgs::Point>("com", 1000);

    // Rosservice
    ss_ = controller_nh.advertiseService("servicesManager", &Controller::servicesManager, this); //FIXME it should be moved to a dedicated interface

    return true;
}

bool Controller::servicesManager(dls_controller::DlsControllerServices::Request &req,
                                 dls_controller::DlsControllerServices::Response &res) // FIXME automatize that function with a list
{
    if(std::strcmp(req.command.c_str(), "toggleSolver") == 0)
    {
        toggleSolver();
        res.response = true;
    }

    return true;
}

void Controller::toggleSolver()
{
    solver_started_=!solver_started_;

    if(solver_started_)
        ROS_INFO("Start the solver");
    else
        ROS_INFO("Stop the solver");
}

void Controller::starting(const ros::Time& time)
{
    ROS_DEBUG("Starting DLS Controller");

    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {
        joint_positions_(i+6) = joint_states_[i].getPosition();
        joint_velocities_(i+6) = joint_states_[i].getVelocity();
        joint_accellerations_(i+6) = 0.0; // FIXME
        joint_efforts_(i+6) = joint_states_[i].getEffort();
    }

    des_joint_positions_ = joint_positions_;
    des_joint_velocities_ = joint_velocities_;

    xbot_model_->setJointPosition(des_joint_positions_);
    xbot_model_->setJointVelocity(des_joint_velocities_);
    //xbot_model_->setJointAcceleration(joint_accellerations_);
    xbot_model_->update();
    ci_->reset(time.toSec());
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
    // Read from the hardware interface
    // This is for the closed loop
    /*for (unsigned int i = 0; i < joint_states_.size(); i++)
    {
        joint_positions_(i) = joint_states_[i].getPosition();
        joint_velocities_(i) = joint_states_[i].getVelocity();
        joint_accellerations_(i) = 0.0; // FIXME
        joint_efforts_(i) = joint_states_[i].getEffort();
    }*/

    if(solver_started_)
    {
        xbot_model_->setJointPosition(des_joint_positions_);
        xbot_model_->setJointVelocity(des_joint_velocities_);
        //xbot_model_->setJointAcceleration(joint_accellerations_);
        xbot_model_->update();

        // Solve IK
        if(!ci_->update(time.toSec(),period.toSec()))
        {
            ROS_ERROR("CartesianInterface: unable to solve");
            return;
        }

        // Integrate solution
        xbot_model_->getJointVelocity(des_joint_velocities_);
        des_joint_positions_ += period.toSec() * des_joint_velocities_;
    }
    else
    {
        des_joint_positions_ = joint_positions_;
        des_joint_velocities_ = joint_velocities_;
    }

    // Write to the hardware interface
    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {
        // use tau to compensate the gravity
        joint_states_[i].setCommandEffort(0.0);
        joint_states_[i].setCommandPosition(des_joint_positions_(i+6));
        joint_states_[i].setCommandVelocity(0.0);
        joint_states_[i].setCommandGains(desired_joint_p_gain_[i], desired_joint_i_gain_[i], desired_joint_d_gain_[i]); //Set Gains P I D
    }

    // Publish
    if(realtime_pub_->trylock())
    {
        for(unsigned int i = 0; i < joint_positions_.size(); i++)
        {
            realtime_pub_->msg_.position[i] = des_joint_positions_(i);
            realtime_pub_->msg_.velocity[i] = des_joint_velocities_(i);
            //realtime_pub_->msg_.effort[i] = joint_efforts_(i);
            realtime_pub_->msg_.header.stamp = time;
            realtime_pub_->unlockAndPublish();
        }
    }

    odomPublisher(); // FIXME move it to a separate thread
}

void Controller::setComReference(const geometry_msgs::Point::ConstPtr& msg)
{
    Eigen::Vector3d ref(msg->x,msg->y,msg->z);
    ci_->setComPositionReference(ref); // FIXME Is it thread safe?
}

void Controller::odomPublisher()
{
    // Get floating base
    Eigen::Affine3d base_pose, world_pose;
    Eigen::Vector3d position;
    Eigen::Quaterniond quaternion;
    xbot_model_->getFloatingBasePose(base_pose); // FIXME Is it thread safe?

    // Do the inverse of it
    world_pose = base_pose.inverse();
    position = world_pose.translation();
    quaternion = world_pose.linear();
    quaternion.normalize();

    // Create the tf transform between /ci/base_link and /ci/world_odom
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(position(0),position(1),position(2)));
    tf::Quaternion q;
    q.setX(quaternion.x());
    q.setY(quaternion.y());
    q.setZ(quaternion.z());
    q.setW(quaternion.w());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/ci/base_link" , "/ci/world_odom" ));

    // Create the tf transform between /ci/base_link and /base_link
    transform.setOrigin(tf::Vector3(0,0,0));
    q.setX(0);
    q.setY(0);
    q.setZ(0);
    q.setW(1);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/ci/base_link", "/base_link"));

    // Publish the com position
    Eigen::Vector3d com;
    geometry_msgs::Point msg;
    ci_->getModel()->getCOM(com); // FIXME Is it thread safe?
    msg.x = com(0);
    msg.y = com(1);
    msg.z = com(2);
    com_pub_.publish(msg);
}

void Controller::stopping(const ros::Time& time)
{
    ROS_DEBUG("Stopping DLS Controller");
}

} //namespace
