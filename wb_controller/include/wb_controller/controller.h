#ifndef WB_CONTROLLER_H
#define WB_CONTROLLER_H

// ROS
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/WrenchStamped.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
// PluginLib
#include <pluginlib/class_list_macros.hpp>
// ROS control
#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
// Hardware interfaces // FIXME Remove that crap
//#include <dls_hardware_interface/joint_command_adv_interface.h>
// ADVR
#include <cartesian_interface/open_sot/OpenSotImpl.h>
#include <cartesian_interface/utils/estimation/ForceEstimation.h>
#include <XBotCoreModel/XBotCoreModel.h>
#include <OpenSoT/floating_base_estimation/qp_estimation.h>
// STD
#include <atomic>
#include <thread>
#include <chrono>
// Controller
#include <wb_controller/locomotion.h>
#include <wb_controller/commands_interface.h>
#include <wb_controller/joy.h>
#include <wb_controller/id_problem.h>
#include <wb_controller/ContactForces.h>
#include <wb_controller/ControllerServices.h>
#include <wb_controller/controllerConfig.h>
#include <wb_controller/Efforts.h>

#include <Eigen/Geometry>

namespace wb_controller
{

class Controller : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                                         hardware_interface::ImuSensorInterface>
{
public:
    /** @brief Constructor function */
    Controller();

    /** @brief Destructor function */
    ~Controller();

    /**
         * @brief Initializes sample controller
         * @param hardware_interface::RobotHW* robot hardware interface
         * @param ros::NodeHandle& Root node handle
         * @param ros::NodeHandle& Supervisor node handle
         */
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    /**
         * @brief Starts the sample controller when controller manager request it
         * @param const ros::Time& time Time
         */
    void starting(const ros::Time& time);

    /**
         * @brief Updates the sample controller according to the control
         * frequency (task frequency)
         * @param const ros::Time& time Time
         * @param const ros::Duration& Period
         */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
         * @brief Stops the sample controller when controller manager request it
         * @param const ros::time& Time
         */
    void stopping(const ros::Time& time);

    /**
         * @brief Ros dynamic reconfigure callback
         */
    void dynamicReconfigureCallback(wb_controller::controllerConfig &config, uint32_t level);

    /**
         * @brief Start/Stop solver integration
         */
    void toggleSolver();

    /**
         * @brief Start/Stop the PIDs
         */
    void togglePid();

    /**
         * @brief Start/Stop the tracking for the tasks
         */
    void toggleTracking();

    /**
         * @brief Start/Stop the haptic contact loop
         */
    void toggleHapticContactLoop();

    /**
         * @brief Set the duty cycle for the feet
         * @param const double duty_cycle
         */
    bool setDutyCycle(const double& duty_cycle);

    /**
         * @brief Set the gait type
         * @param const std::string& gait_type
         */
    bool setGaitType(const std::string& gait_type);

    /**
         * @brief Set the swing frequency
         * @param const double& swing_frequency
         */
    bool setSwingFrequency(const double& swing_frequency);

private:

    /** @brief Number of joints */
    unsigned int num_joints_;
    /** @brief Joint names */
    std::vector<std::string> joint_names_;
    /** @brief Imu sensor name */
    std::string imu_name_;
    /** @brief Joint states for reading */
    //std::vector<hardware_interface::JointStateHandle> joint_states_;
    /** @brief Joint states for reading positions, velocities and efforts and writing effort commands */
    std::vector<hardware_interface::JointHandle> joint_states_;
    /** @brief IMU sensors */
    hardware_interface::ImuSensorHandle imu_sensor_;
    /** @brief Joint positions */
    Eigen::VectorXd joint_positions_;
    /** @brief Joint velocities */
    Eigen::VectorXd joint_velocities_;
    /** @brief Joint accellerations */
    Eigen::VectorXd joint_accellerations_;
    /** @brief Joint efforts */
    Eigen::VectorXd joint_efforts_;
    /** @brief Desired joint positions */
    Eigen::VectorXd des_joint_positions_;
    /** @brief Desired joint velocities */
    Eigen::VectorXd des_joint_velocities_;
    /** @brief Desired joint efforts sent to the hardware interface */
    Eigen::VectorXd des_joint_efforts_;
    /** @brief Desired joint efforts computed by the solver */
    Eigen::VectorXd des_joint_efforts_solver_;
    /** @brief Desired joint efforts computed by the PIDs */
    Eigen::VectorXd des_joint_efforts_pids_;
    /** @brief Solver's solution (i.e. efforts) */
    Eigen::VectorXd x_;
    /** @brief Xbot robot model */
    XBot::ModelInterface::Ptr xbot_model_;
    /** @brief Dynamic problem formulation */
    OpenSoT::IDProblem::Ptr id_prob_;
    /** @brief Base estimation */
    OpenSoT::floating_base_estimation::qp_estimation::Ptr qp_estimation_;
    /** @brief Contact estimation */
    XBot::Cartesian::Utils::ForceEstimation::Ptr force_estimation_;
    /** @brief Contact estimation */
    std::vector<XBot::ForceTorqueSensor::ConstPtr> force_torque_sensors_;
    /** @brief Real time publisher - desired joint states */
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> ci_joint_states_rt_pub_;
    /** @brief Real time publisher - IMU */
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu>> imu_rt_pub_;
    /** @brief Real time publisher - estimated pose */
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> state_estimation_rt_pub_; // FIXME to be removed
    /** @brief Real time publisher - estimated qp pose */
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> state_estimation_qp_rt_pub_;
    /** @brief Real time publisher - contact forces */
    std::shared_ptr<realtime_tools::RealtimePublisher<wb_controller::ContactForces>> contact_forces_pub_;
    /** @brief Real time publisher - Efforts */
    std::shared_ptr<realtime_tools::RealtimePublisher<wb_controller::Efforts>> efforts_pub_;
    /** @brief Ros subscriber for the desired tasks reference */
    ros::Subscriber tasks_desired_sub_;
    /** @brief Desired P value for the joints PID controller */
    std::vector<double> des_joint_p_gain_;
    /** @brief Desired I value for the joints PID controller */
    std::vector<double> des_joint_i_gain_;
    /** @brief Desired D value for the joints PID controller */
    std::vector<double> des_joint_d_gain_;
    /** @brief Actual P value for the joints PID controller */
    std::vector<double> joint_p_gain_;
    /** @brief Actual I value for the joints PID controller */
    std::vector<double> joint_i_gain_;
    /** @brief Actual D value for the joints PID controller */
    std::vector<double> joint_d_gain_;
    /** @brief Vector containing the pids for the joints */
    std::vector<control_toolbox::Pid> pids_;
    /** @brief Integrate the solver solution and apply it to the desired joints state */
    std::atomic<bool> solver_started_;
    /** @brief Contact force threshold, this is a normalized value. The actual contact force get compared to this value and if greater equal the contact
    is consired true */
    std::atomic<double> contact_force_th_;
    /** @brief Activate pid gains */
    std::atomic<bool> pid_active_;
    /** @brief Activate tracking */
    std::atomic<bool> tracking_active_;
    /** @brief Activate the contact haptic loop */
    std::atomic<bool> haptic_contact_loop_;
    /** @brief Variable used to signal that the controller is stopping */
    std::atomic<bool> stopping_;
    /** @brief ROS dynamic reconfigure */
    dynamic_reconfigure::Server<wb_controller::controllerConfig>* server_;
    /** @brief ROS dynamic reconfigure config struct */
    controllerConfig default_config_;
    /** @brief IMU Accelerometer */
    Eigen::Vector3d imu_accelerometer_;
    /** @brief IMU Gyroscope */
    Eigen::Vector3d imu_gyroscope_;
    /** @brief IMU Orientation */
    Eigen::Quaterniond imu_orientation_;
    /** @brief Homing position, loaded from the srdf file */
    Eigen::VectorXd qhome_;
    /** @brief Ground reaction forces generated by the solver  */
    Eigen::VectorXd des_contact_forces_;
    /** @brief Floating base position w.r.t the world frame, computed by the state estimator */
    Eigen::Vector3d floating_base_position_;
    /** @brief Floating base orientation w.r.t the world frame, computed by the state estimator (RPY) */
    Eigen::Vector3d floating_base_orientation_rpy_;
    /** @brief Floating base orientation w.r.t the world frame, computed by the state estimator */
    Eigen::Quaterniond floating_base_orientation_;
    /** @brief Floating base velocity, computed by the state estimator */
    Eigen::Vector6d floating_base_velocity_;
    /** @brief Floating base velocity, computed by the QP */
    Eigen::VectorXd floating_base_velocity_qp_;
    /** @brief Floating base pose w.r.t the world frame, computed by the state estimator */
    Eigen::Affine3d floating_base_pose_;    
    /** @brief GRF contacts */
    std::vector<bool> contacts_;
    /** @brief GRF contact forces */
    std::vector<Eigen::Vector3d> contact_forces_;
    /** @brief Feet names */
    std::vector<std::string> feet_names_;
    /** @brief Feet positions w.r.t world */
    std::vector<Eigen::Vector3d> world_X_foot_;
    /** @brief Arm tip name */
    std::string arm_tip_name_;
    /** @brief Hips names */
    std::vector<std::string> hips_names_;
    /** @brief Thread for the odometry publisher */
    std::shared_ptr<std::thread> odom_publisher_thread_;
    /** @brief Thread for the rviz publisher */
    std::shared_ptr<std::thread> rviz_publisher_thread_;
    /** @brief True if the solver has been resetted */
    bool solver_reset_done_;
    /** @brief Align the imu frame (trunk) to the world */
    bool imu_reset_done_;
    /** @brief Gait generator */
    wb_controller::GaitGenerator::Ptr gait_generator_;
    /** @brief Visual tools */
    std::map<std::string,rviz_visual_tools::RvizVisualToolsPtr> visual_tools_;
    /** @brief Ros node handle */
    ros::NodeHandle nh_;
    /** @brief Joy handler */
    JoyHandler::Ptr joy_handler_;
    /** @brief Command interface */
    CommandsInterface::Ptr cmds_;
    /** @brief Initial rotation of the imu w.r.t world */
    Eigen::Matrix3d world_R_imu_;
    /** @brief Support temporary Affine3d */
    Eigen::Affine3d tmp_affine3d_;
    /** @brief Support temporary Vector3d */
    Eigen::Vector3d tmp_vector3d_;
    /** @brief Support temporary Matrix3d */
    Eigen::Matrix3d tmp_matrix3d_;

    ros::ServiceClient freeze_base_client;

    /**
         * @brief thread body for the odometry publisher
         */
    void odomPublisher();

    /**
         * @brief thread body for the rviz tools publisher
         */
    void rvizPublisher();

    /**
         * @brief update the joints state
         */
    void readJoints();

    /**
         * @brief update the imu reading from the imu interface
         */
    void readImu();

    /**
         * @brief update floating base state
         */
    void stateEstimation();

    /**
         * @brief update the virtual model
         */
    void updateXBotModel();

    /**
         * @brief update the contacts state
         */
    void readContactsState();

    /**
         * @brief init the ROS publishers
         */
    void initPublishers(const ros::NodeHandle& root_nh, const ros::NodeHandle& controller_nh);

    /**
         * @brief publish on ROS
         */
    void publish(const ros::Time& time, const ros::Duration& period);

    /**
         * @brief Update the dynamic reconfigure interface
         */
    void dynamicReconfigureUpdate();

};


PLUGINLIB_EXPORT_CLASS(wb_controller::Controller, controller_interface::ControllerBase);

} //@namespace wb_controller

#endif //WB_CONTROLLER_H
