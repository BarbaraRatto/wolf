#ifndef DLS_HW_SIM_H
#define DLS_HW_SIM_H

// ROS includes
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// Gazebo includes
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/ContactSensor.hh>

// Gazebo ros control include
#include <gazebo_ros_control/robot_hw_sim.h>

// URDF include
#include <urdf/model.h>

// DLs HW interface
#include <dls_hardware_interface/dls_robot_hw.h>

namespace dls_gazebo_interface
{

/**
 * @class This class implements the Gazebo hardware interface of HyQ2Max.
 * @brief This hardware interface is loaded to Gazebo using gazebo_ros_control plugin
 * which required the initSim, readSim and writeSim methods override in this class.
 */
class DlsRobotHwSim : public gazebo_ros_control::RobotHWSim, public hardware_interface::DlsRobotHwInterface
{
public:
  /**
     * @brief Initializes the Hy2Max hardware interface by reading the urdf file
     * @param const std::string& robot_namespace Robot namespace
     * @param ros::NodeHandle Model node handle
     * @param gazebo::physics::ModelPtr Gazebo model pointer
     * @param const urdf::Model *const URDF model
     * @param std::vector<transmission_interface::TransmissionInfo> Transmissions information
     */
  bool initSim(const std::string& robot_namespace,
               ros::NodeHandle model_nh,
               gazebo::physics::ModelPtr parent_model,
               const urdf::Model *const urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions);

  /**
     * @brief Reads all the sensors of HyQ2Max from Gazebo: encoders and imu
     * @param ros::Time Simulated time
     * @param ros::Duration Simulated period
     */
  void readSim(ros::Time time, ros::Duration period);

  /**
     * @brief Writes the forces values to Gazebo
     * @param ros::Time Simulated time
     * @param ros::Duration Simulated period
     */
  void writeSim(ros::Time time, ros::Duration period);

  /**
     * @brief Handles the user commands to control the simulation environment
     */
  void simulationInterface(void);


private:

  std::shared_ptr<gazebo::sensors::ImuSensor> imu_sensor_;
  std::vector<std::shared_ptr<gazebo::sensors::ContactSensor> > foot_sensors_;
  std::vector<std::shared_ptr<gazebo::sensors::ContactSensor> > shin_sensor_;
  //hardware_interface::ShinSensorInterface shin_sensor_interface_;
  //std::deque<bool> shin_contact_position_;

  std::vector<gazebo::physics::JointPtr> sim_joints_;
  gazebo::physics::ModelPtr sim_model_;
  gazebo::math::Pose inital_pose;

  std::vector<double> torque_offsets;
  bool useTorqueOffsets;

};

typedef std::shared_ptr<DlsRobotHwSim> DlsRobotHwSimPtr;

}

#endif
