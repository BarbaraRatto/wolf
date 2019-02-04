#ifndef HYQ_DUMMY_H
#define HYQ_DUMMY_H

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

// ros_control
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

// DLs HW interface
#include <dls_hardware_interface/dls_robot_hw.h>

// NaN
#include <limits>

// ostringstream
#include <sstream>

class HyqDummy : public hardware_interface::RobotHW, public hardware_interface::DlsRobotHwInterface
{

public:
  HyqDummy()
    : running_(true)
    , start_srv_(nh_.advertiseService("start", &HyqDummy::start_callback, this))
    , stop_srv_(nh_.advertiseService("stop", &HyqDummy::stop_callback, this))
  {

  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}

  void init(const std::string& joints_param_name)
  {

     // Get joint names from the parameter server, using the controller config file
    using namespace XmlRpc;
    XmlRpcValue joint_names;
    if (!nh_.getParam(joints_param_name, joint_names))
    {
    ROS_ERROR_STREAM("No joints given (expected namespace: /" + joints_param_name + ").");
    }
    if (joint_names.getType() != XmlRpcValue::TypeArray)
    {
    ROS_ERROR_STREAM("Malformed joint specification (namespace: /" + joints_param_name + ").");
    }
    else
        ROS_INFO_STREAM("Loaded (namespace: /" + joints_param_name + ").");

    n_dofs_ = joint_names.size();

   /* joints_t joints_tmp; // Temporary container of joint handles
    jointNames_t jointNames_tmp; // Temporary container of joint names

    joints_tmp.resize(joint_names.size());
    jointNames_tmp.resize(joint_names.size());*/

    // Defining the robot name
    //robot_name_ = robot_name;

    // Resizing the joint vectors
    joint_names_.resize(n_dofs_);
    joint_position_.resize(n_dofs_);
    joint_velocity_.resize(n_dofs_);
    joint_effort_.resize(n_dofs_);
    joint_p_gain_.resize(n_dofs_);
    joint_i_gain_.resize(n_dofs_);
    joint_d_gain_.resize(n_dofs_);
    joint_position_command_.resize(n_dofs_);
    joint_velocity_command_.resize(n_dofs_);
    joint_effort_command_.resize(n_dofs_);
    joint_p_gain_command_.resize(n_dofs_);
    joint_i_gain_command_.resize(n_dofs_);
    joint_d_gain_command_.resize(n_dofs_);

    // Intialize raw data
    //joint_position_.fill(0.0);
    /*std::fill_n(joint_velocity_, n_dofs_, 0.0);
    std::fill_n(joint_effort_, n_dofs_, 0.0);
    std::fill_n(joint_position_command_, n_dofs_, 0.0);
    std::fill_n(joint_velocity_command_, n_dofs_, 0.0);
    std::fill_n(joint_effort_command_, n_dofs_, 0.0);*/

    //char name[100];
    for (int i = 0; i < n_dofs_; i++) {   
          XmlRpcValue &name_value = joint_names[i];
	  if (name_value.getType() != XmlRpcValue::TypeString)
	  {
	      ROS_ERROR_STREAM("Array of joint names should contain all strings (namespace:" << nh_.getNamespace() << ").");
	  }
	  const std::string joint_name = static_cast<std::string>(name_value);

          joint_names_[i] = joint_name;
          joint_position_[i] = 1.0;
          joint_velocity_[i] = 0.0;
          joint_effort_[i] = 0.0;
          joint_p_gain_[i] = 0.0;
          joint_i_gain_[i] = 0.0;
          joint_d_gain_[i] = 0.0;
          joint_position_command_[i] = 0.0;
          joint_velocity_command_[i] = 0.0;
          joint_effort_command_[i] = 0.0;
          joint_p_gain_command_[i] = 0.0;
          joint_i_gain_command_[i] = 0.0;
          joint_d_gain_command_[i] = 0.0;

          // Connect and register the joint state interface
          joint_state_adv_interface_.registerHandle(hardware_interface::JointStateAdvHandle(
              joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i],
              &joint_p_gain_[i], &joint_i_gain_[i], &joint_d_gain_[i]));

          joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
              joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

          joint_interface_.registerHandle(hardware_interface::JointCommandAdvHandle(
              joint_state_adv_interface_.getHandle(joint_names_[i]), &joint_position_command_[i],
              &joint_velocity_command_[i], &joint_effort_command_[i],
              &joint_p_gain_command_[i], &joint_i_gain_command_[i], &joint_d_gain_command_[i]));

          joint_effort_command_[i] = 0;
    }

    imu_orientation_.resize(4);
    imu_ang_vel_.resize(3);
    imu_lin_acc_.resize(3);

    base_orientation_.resize(4);
    base_ang_vel_.resize(3);
    base_ang_acc_.resize(3);
    base_lin_acc_.resize(3);
    base_lin_pos_.resize(3);
    base_lin_vel_.resize(3);

    imuData.name = "trunk_imu"; // TODO: Fetch from elsewhere?
    imuData.frame_id = "base_link"; // TODO: Fetch from URDF?
    imuData.orientation = &imu_orientation_[0];
    imuData.angular_velocity = &imu_ang_vel_[0];
    imuData.linear_acceleration = &imu_lin_acc_[0];
    imu_sensor_interface_.registerHandle(hardware_interface::ImuSensorHandle(imuData));


    gtData.name = "ground_truth"; // TODO: Fetch from elsewhere?
    gtData.frame_id = "base_link"; // TODO: Fetch from URDF?
    gtData.orientation = &base_orientation_[0];
    gtData.angular_velocity = &base_ang_vel_[0];
    gtData.angular_acceleration = &base_ang_acc_[0];
    gtData.linear_acceleration = &base_lin_acc_[0];
    gtData.linear_position = &base_lin_pos_[0];
    gtData.linear_velocity = &base_lin_vel_[0];
    ground_truth_interface_.registerHandle(hardware_interface::GroundTruthHandle(gtData));

    leg_name_.resize(4);
    force_.resize(4);
    torque_.resize(4);
    contact_.resize(4);

    std::string frame("base_link"); //Some quick hack to pass the data over;

    std::string name_str0("lf");
    leg_name_[0] = name_str0;
    std::string name_str1("rf");
    leg_name_[1] = name_str1;
    std::string name_str2("lh");
    leg_name_[2] = name_str2;
    std::string name_str3("rh");
    leg_name_[3] = name_str3;


    for (int n = 0; n < 4; n++) {
      force_[n].resize(3);
      torque_[n].resize(3);
      force_sensor_interface_.registerHandle(hardware_interface::ForceTorqueSensorHandle(
        leg_name_[n], frame, &force_[n][0], &torque_[n][0]));

      contact_sensor_interface_.registerHandle(hardware_interface::ContactSwitchSensorHandle(
                leg_name_[n], &contact_[n]));
    }

    xenomai_switch_count_sim_ = -2; // Random value
    std::string sim_interface_name("sim"); //Some quick hack to pass the data over;
    sim_interface_.registerHandle(hardware_interface::SimulationHandle(sim_interface_name,
      &xenomai_switch_count_sim_,&is_robot_real_sim_,&pause_sim_,&reset_sim_,&freeze_base_sim_,
      &ext_force_sim_, &ext_torque_sim_));


    /*for(int i = 0; i < (N_ROBOT_MISC_SENSORS -1); i++)
    {
      std::string misc = stringFromMiscSensor(static_cast<RobotMiscSensors>(i));
      misc_sensors_.push_back(misc_sensor[i+1]);
      misc_sensors_names_.push_back(misc);
    }*/
    std::string motor_interface_name("motor");
    motor_interface_.registerHandle(hardware_interface::MotorHandle(motor_interface_name,
      &remove_motor_torque_offsets_, &misc_sensors_, &misc_sensors_names_));

    registerInterface(&joint_state_adv_interface_);
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_interface_);
    registerInterface(&imu_sensor_interface_);
    registerInterface(&ground_truth_interface_);
    registerInterface(&force_sensor_interface_);
    registerInterface(&contact_sensor_interface_);
    registerInterface(&sim_interface_);
    registerInterface(&motor_interface_);

  }

  void read()
  {
    std::ostringstream os;
    for (unsigned int i = 0; i < n_dofs_ - 1; ++i)
    {
      os << joint_effort_command_[i] << ", ";
    }
    os << joint_effort_command_[n_dofs_ - 1];

    ROS_DEBUG_STREAM("Effort commands for joints: " << os.str());
  }

  void write()
  {
    if (running_)
    {
      for (unsigned int i = 0; i < n_dofs_; ++i)
      {
        // Note that pos_[i] will be NaN for one more cycle after we start(),
        // but that is consistent with the knowledge we have about the state
        // of the robot.


        // Do some amazing stuff
      }
    }
    else
    {
      //std::fill_n(joint_position_, n_dofs_, std::numeric_limits<double>::quiet_NaN());
      //std::fill_n(joint_velocity_, n_dofs_, std::numeric_limits<double>::quiet_NaN());
      //std::fill_n(joint_effort_, n_dofs_, std::numeric_limits<double>::quiet_NaN());
    }
  }

  void freezeBase(int flag){}

  bool start_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = false;
    return true;
  }

private:

  bool running_;

  ros::NodeHandle nh_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
  int n_dofs_;
};

#endif
