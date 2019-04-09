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

#define FLOATING_BASE_DOFS 6
#define CONTROLLER_NAME "dls_controller"

Controller::Controller()
    :solver_started_(false)
    ,gravity_compensation_(true)
    ,pid_active_(true)
    ,stopping_(false)
{
}

Controller::~Controller()
{
    if(realtime_pub_)
        delete realtime_pub_;
}

bool Controller::init(hardware_interface::RobotHW* robot_hw,
                      ros::NodeHandle& root_nh,
                      ros::NodeHandle& controller_nh)
{
    // getting the names of the joints from the ROS parameter server
    ROS_DEBUG("Initialize DLS Controller");

    assert(robot_hw);

    hardware_interface::JointCommandAdvInterface* jt_hw = robot_hw->get<hardware_interface::JointCommandAdvInterface>();
    hardware_interface::ImuSensorInterface* imu_hw = robot_hw->get<hardware_interface::ImuSensorInterface>();

    if(!jt_hw)
    {
        ROS_ERROR("hardware_interface::JointCommandAdvInterface not found");
        return false;
    }
    if(!imu_hw)
    {
        ROS_ERROR("hardware_interface::ImuSensorInterface not found");
        return false;
    }

    if (!controller_nh.getParam("joints", joint_names_))
    {
        ROS_ERROR("No joints given in the namespace: %s.", controller_nh.getNamespace().c_str());
        return false;
    }

    if (!controller_nh.getParam("imu_sensors", imu_names_))
    {
        ROS_ERROR("No imu_sensors given in the namespace: %s.", controller_nh.getNamespace().c_str());
        return false;
    }

    // Setting up handles:
    for (unsigned int i = 0; i < joint_names_.size(); i++)
    {
        // Getting joint state handle
        try
        {
            ROS_DEBUG_STREAM("Found joint: "<<joint_names_[i]);
            joint_states_.push_back(jt_hw->getHandle(joint_names_[i]));
        }
        catch(...)
        {
            ROS_ERROR("Error loading joint_states_");
            return false;
        }
    }
    assert(joint_states_.size()>0);

    for (unsigned int i = 0; i < imu_names_.size(); i++)
    {
        try
        {
            ROS_DEBUG_STREAM("Found imu sensor: "<<imu_names_[i]);
            imu_sensors_.push_back(imu_hw->getHandle(imu_names_[i]));
        }
        catch(...)
        {
            ROS_ERROR("Error loading imu_sensors_");
            return false;
        }
    }
    assert(imu_sensors_.size()>0);

    des_joint_p_gain_.resize(joint_states_.size());
    des_joint_i_gain_.resize(joint_states_.size());
    des_joint_d_gain_.resize(joint_states_.size());
    joint_p_gain_.resize(joint_states_.size());
    joint_i_gain_.resize(joint_states_.size());
    joint_d_gain_.resize(joint_states_.size());
    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {
        // Getting PID gains
        if (!controller_nh.getParam("gains/" + joint_names_[i] + "/p", joint_p_gain_[i]))
        {
            ROS_ERROR("No P gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
            return false;
        }
        if (!controller_nh.getParam("gains/" + joint_names_[i] + "/i", joint_i_gain_[i]))
        {
            ROS_ERROR("No D gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
            return false;
        }
        if (!controller_nh.getParam("gains/" + joint_names_[i] + "/d", joint_d_gain_[i]))
        {
            ROS_ERROR("No I gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
            return false;
        }
        // Check if the values are positive
        if(joint_p_gain_[i]<0.0 || joint_i_gain_[i]<0.0 || joint_d_gain_[i]<0.0)
        {
            ROS_ERROR("PID gains must be positive!");
            return false;
        }
        ROS_DEBUG("P value for joint %i is: %d",i,joint_p_gain_[i]);
        ROS_DEBUG("I value for joint %i is: %d",i,joint_i_gain_[i]);
        ROS_DEBUG("D value for joint %i is: %d",i,joint_d_gain_[i]);

        // Set the gain value when the error is 0 and the gain value when the error reach x [m]
        //double x = 0.1;
        //adaptive_joint_p_gain_.push_back(new AdaptiveGain(des_joint_p_gain_[i],des_joint_p_gain_[i]/2.0,x));
    }

    // Create the ModelInterface from XBot
    XBot::ConfigOptions opt;
    std::string urdf, srdf, problem;

    if(!root_nh.getParam("/robot_description",urdf)) // Get the robot description from the global namespace "/"
    {
        ROS_ERROR_STREAM_NAMED(CONTROLLER_NAME,"No robot_description given in namespace /");
        return false;
    }
    if(!root_nh.getParam("/robot_semantic_description",srdf)) // Get the robot semantic description from the global namespace "/"
    {
        ROS_ERROR_STREAM_NAMED(CONTROLLER_NAME,"No robot_semantic_description given in namespace /");
        return false;
    }
    if(!opt.set_urdf(urdf))
    {
        ROS_ERROR("Unable to load urdf");
        return false;
    }
    if(!opt.set_srdf(srdf))
    {
        ROS_ERROR("Unable to load srdf");
        return false;
    }
    if(!opt.generate_jidmap())
    {
        ROS_ERROR("Unable to load jidmap");
        return false;
    }
    opt.set_parameter("is_model_floating_base", true);
    opt.set_parameter<std::string>("model_type", "RBDL");
    xbot_model_ = XBot::ModelInterface::getModel(opt);

    // Set home position defined in the srdf
    xbot_model_->getRobotState("home", qhome_);
    xbot_model_->setJointPosition(qhome_);

    // Create the kinematics problem
    if(!controller_nh.getParam("problem_description",problem))
    {
        ROS_ERROR("No problem_description given");
        return false;
    }
    YAML::Node config = YAML::Load(problem);
    ProblemDescription ik_problem(config, xbot_model_);

    // Create the CartesianInterfaceImpl from XBot::Cartesian
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

    // Create the force optimization to compute the gravity compensation terms
    std::vector<std::string> contact_links;

    // Those are associated to the SRDF model
    contact_links.push_back("rh_foot");
    contact_links.push_back("rf_foot");
    contact_links.push_back("lh_foot");
    contact_links.push_back("lf_foot");
    fo_.reset(new OpenSoT::utils::ForceOptimization(xbot_model_,contact_links,false));

    // Resize the variables
    joint_positions_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    joint_velocities_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    joint_accellerations_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    joint_efforts_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    des_joint_positions_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    des_joint_velocities_.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    des_joint_efforts_.resize(joint_states_.size()+FLOATING_BASE_DOFS);

    joint_positions_.fill(0.0);
    joint_velocities_.fill(0.0);
    joint_accellerations_.fill(0.0);
    joint_efforts_.fill(0.0);
    com_position_.fill(0.0);
    des_joint_positions_.fill(0.0);
    des_joint_velocities_.fill(0.0);
    des_joint_efforts_.fill(0.0);
    des_com_position_.fill(0.0);

    // Create the realtime publishers
    realtime_pub_ = new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "ci/joint_states", 4);
    realtime_pub_->msg_.name.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    realtime_pub_->msg_.position.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    realtime_pub_->msg_.velocity.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    realtime_pub_->msg_.effort.resize(joint_states_.size()+FLOATING_BASE_DOFS);
    realtime_pub_->msg_.name[0] = "x"; //FIXME
    realtime_pub_->msg_.name[1] = "y";
    realtime_pub_->msg_.name[2] = "z";
    realtime_pub_->msg_.name[3] = "r";
    realtime_pub_->msg_.name[4] = "p";
    realtime_pub_->msg_.name[5] = "y";
    for (unsigned int i = 0; i < joint_names_.size(); i++)
        realtime_pub_->msg_.name[i+FLOATING_BASE_DOFS] = joint_names_[i];

    // Reference for the com
    com_ref_sub_ = controller_nh.subscribe("com_ref", 1, &Controller::setComReference, this);
    // Position of the com
    com_pub_ = controller_nh.advertise<geometry_msgs::Point>("com", 1000);

    // Rosservice
    ss_ = controller_nh.advertiseService("servicesManager", &Controller::servicesManager, this); //FIXME it should be moved to a dedicated interface

    // Spawn the odom publisher thread
    odom_publisher_thread_.reset(new std::thread(&Controller::odomPublisher,this));

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
    if(std::strcmp(req.command.c_str(), "toggleGravityCompensation") == 0)
    {
        toggleGravityCompensation();
        res.response = true;
    }
    if(std::strcmp(req.command.c_str(), "togglePid") == 0)
    {
        togglePid();
        res.response = true;
    }

    return true;
}

void Controller::togglePid()
{
    pid_active_=!pid_active_;

    if(pid_active_)
        ROS_INFO("PIDs are ON");
    else
        ROS_INFO("PIDs are OFF");
}

void Controller::toggleSolver()
{
    solver_started_=!solver_started_;

    if(solver_started_)
        ROS_INFO("Solver integration is ON");
    else
        ROS_INFO("Solver integration is OFF");
}

void Controller::toggleGravityCompensation()
{
    gravity_compensation_=!gravity_compensation_;

    if(gravity_compensation_)
        ROS_INFO("Gravity compensation is ON");
    else
        ROS_INFO("Gravity compensation is OFF");
}

void Controller::readImu()
{
    // FIXME For now we select the first imu
    unsigned int selected_imu = 0;
    imu_accelerometer_ = Eigen::Map<const Eigen::Vector3d>(imu_sensors_[selected_imu].getLinearAcceleration());
    imu_gyroscope_ = Eigen::Map<const Eigen::Vector3d>(imu_sensors_[selected_imu].getAngularVelocity());
    imu_orientation_.w() = imu_sensors_[selected_imu].getOrientation()[0];
    imu_orientation_.x() = imu_sensors_[selected_imu].getOrientation()[1];
    imu_orientation_.y() = imu_sensors_[selected_imu].getOrientation()[2];
    imu_orientation_.z() = imu_sensors_[selected_imu].getOrientation()[3];
}

void Controller::readJoints()
{
    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {
        joint_positions_(i+FLOATING_BASE_DOFS) = joint_states_[i].getPosition();
        joint_velocities_(i+FLOATING_BASE_DOFS) = joint_states_[i].getVelocity();
        joint_accellerations_(i+FLOATING_BASE_DOFS) = 0.0; // FIXME
        joint_efforts_(i+FLOATING_BASE_DOFS) = joint_states_[i].getEffort();
    }
}

void Controller::starting(const ros::Time& time)
{
    ROS_DEBUG("Starting DLS Controller");

    // Read from the hardware interfaces:
    // 1) Joints
    readJoints();
    // 2) IMU
    readImu();

    des_joint_positions_ = joint_positions_;
    des_joint_velocities_ = joint_velocities_;
    des_joint_efforts_.fill(0.0);
    xbot_model_->getCOM(com_position_);

    xbot_model_->setJointPosition(des_joint_positions_);
    xbot_model_->setJointVelocity(des_joint_velocities_);
    //xbot_model_->setJointAcceleration(joint_accellerations_);
    xbot_model_->update();
    ci_->reset(time.toSec());
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
    // Read from the hardware interfaces:
    // 1) Joints
    // readJoints(); // FIXME For the moment we don't close the loop
    // 2) IMU
    readImu();

    xbot_model_->setJointPosition(des_joint_positions_);
    xbot_model_->setJointVelocity(des_joint_velocities_);
    xbot_model_->setFloatingBaseOrientation(imu_orientation_.normalized().toRotationMatrix().transpose());
    xbot_model_->update();

    if(gravity_compensation_)
    {
        xbot_model_->computeGravityCompensation(des_joint_efforts_); // Compute the g term for the legs and the virtual force applied to the base
        Eigen::VectorXd tau;
        std::vector<Eigen::Vector6d> Fc; // FIXME NO RT!
        if(!fo_->compute(des_joint_efforts_,Fc,tau))
        {
            ROS_ERROR("ForceOptimization: unable to solve");
            return;
        }
        des_joint_efforts_ = tau;
    }
    else
        des_joint_efforts_.fill(0.0);

    if(pid_active_) // FIXME abrupt
    {
        for (unsigned int i = 0; i < des_joint_p_gain_.size(); i++)
        {
            des_joint_p_gain_[i] = joint_p_gain_[i];
            des_joint_i_gain_[i] = joint_i_gain_[i];
            des_joint_d_gain_[i] = joint_d_gain_[i];
        }
    }
    else
    {
        for (unsigned int i = 0; i < des_joint_p_gain_.size(); i++)
        {
            des_joint_p_gain_[i] = 0.0;
            des_joint_i_gain_[i] = 0.0;
            des_joint_d_gain_[i] = 0.0;
        }
    }

    if(solver_started_)
    {
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
        des_com_position_ = com_position_;
        des_joint_positions_ = qhome_;
        des_joint_velocities_.setZero();
    }

    // Write to the hardware interface
    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {
        // use tau to compensate the gravity
        joint_states_[i].setCommandEffort(des_joint_efforts_(i+FLOATING_BASE_DOFS));
        joint_states_[i].setCommandPosition(des_joint_positions_(i+FLOATING_BASE_DOFS));
        joint_states_[i].setCommandVelocity(0.0);
        joint_states_[i].setCommandGains(des_joint_p_gain_[i],des_joint_i_gain_[i], des_joint_d_gain_[i]); //Set Gains P I D
        //joint_states_[i].setCommandGains(adaptive_joint_p_gain_[i]->ComputeGain(des_joint_efforts_(i+FLOATING_BASE_DOFS)-joint_positions_(i+FLOATING_BASE_DOFS))
        //                                 ,des_joint_i_gain_[i], des_joint_d_gain_[i]); //Set Gains P I D
    }

    // Publish
    if(realtime_pub_->trylock())
    {
        for(unsigned int i = 0; i < joint_positions_.size(); i++)
        {
            realtime_pub_->msg_.position[i] = des_joint_positions_(i);
            realtime_pub_->msg_.velocity[i] = des_joint_velocities_(i);
            realtime_pub_->msg_.effort[i] = des_joint_efforts_(i);
            realtime_pub_->msg_.header.stamp = time;
            realtime_pub_->unlockAndPublish();
        }
    }

    //odomPublisher(); // FIXME move it to a separate thread
}

void Controller::setComReference(const geometry_msgs::Point::ConstPtr& msg)
{
    des_com_position_(0) = msg->x;
    des_com_position_(1) = msg->y;
    des_com_position_(2) = msg->z;
    ci_->setComPositionReference(des_com_position_); // FIXME Is it thread safe?
}

void Controller::odomPublisher()
{

    ROS_INFO("Start the odomPublisher");
    while(!stopping_)
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
        geometry_msgs::Point msg;
        xbot_model_->getCOM(com_position_); // FIXME Is it thread safe?
        msg.x = com_position_(0);
        msg.y = com_position_(1);
        msg.z = com_position_(2);
        com_pub_.publish(msg);

        std::this_thread::sleep_for( std::chrono::milliseconds(4) );

    }
    ROS_INFO("Stop the odomPublisher");
}

void Controller::stopping(const ros::Time& time)
{
    ROS_DEBUG("Stopping DLS Controller");

    stopping_ = true;
    odom_publisher_thread_->join();

}

} //namespace
