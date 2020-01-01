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
// PluginLib
#include <pluginlib/class_list_macros.hpp>
// ROS control
#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <dls_hardware_interface/ground_truth_interface.h>
// ADVR
#include <cartesian_interface/open_sot/OpenSotImpl.h>
#include <XBotCoreModel/XBotCoreModel.h>
// STD
#include <atomic>
#include <thread>
#include <chrono>
// Controller
#include <wb_controller/locomotion.h>
#include <wb_controller/footholds_planner.h>
#include <wb_controller/state_estimator.h>
#include <wb_controller/device_interface.h>
#include <wb_controller/id_problem.h>
#include <wb_controller/ContactForces.h>
#include <wb_controller/controllerConfig.h>
// Eigen
#include <Eigen/Geometry>

namespace wb_controller
{

class Controller : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                                         hardware_interface::ImuSensorInterface,
                                                                         hardware_interface::GroundTruthInterface>
{
public:

    /**
     * @brief Shared pointer to Controller
     */
    typedef std::shared_ptr<Controller> Ptr;

    /**
     * @brief Shared pointer to const Controller
     */
    typedef std::shared_ptr<const Controller> ConstPtr;


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
         * @brief Start/Stop the haptic contact loop
         */
    void toggleHapticContactLoop();

    /**
         * @brief Start/Stop the base height control
         */
    void toggleBaseHeightControl();

    /**
         * @brief Start/Stop the inertia compensation at the leg level, useful if the robot has very low inertia at the knee joints
         */
    void toggleInertiaCompensation();

    /**
         * @brief Set the duty factor for the feet
         * @param const double duty_factor
         */
    bool setDutyFactor(const double& duty_factor);

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

    /**
         * @brief Get the gait generator pointer
         */
    GaitGenerator* getGaitGenerator() const;

    /**
         * @brief Get the gait commands interface pointer
         */
    FootholdsPlanner* getFootholdsPlanner() const;

    /**
         * @brief Get the state estimator pointer
         */
    StateEstimator* getStateEstimator() const;

private:

    /** @brief Joint names */
    std::vector<std::string> joint_names_;
    /** @brief Joint states for reading positions, velocities and efforts and writing effort commands */
    std::vector<hardware_interface::JointHandle> joint_states_;
    /** @brief Control period */
    double period_;
    /** @brief IMU sensor name */
    std::string imu_name_;
    /** @brief IMU sensors */
    hardware_interface::ImuSensorHandle imu_sensor_;
    /** @brief Ground Thruth */
    hardware_interface::GroundTruthHandle ground_truth_;
    /** @brief Joint positions */
    Eigen::VectorXd joint_positions_;
    /** @brief Joint velocities */
    Eigen::VectorXd joint_velocities_;
    /** @brief Joint velocities */
    Eigen::VectorXd joint_velocities_filt_;
    /** @brief Joint accellerations */
    Eigen::VectorXd joint_accellerations_;
    /** @brief Joint efforts */
    Eigen::VectorXd joint_efforts_;
    /** @brief XBOT joint positions */
    Eigen::VectorXd joint_positions_xbot_;
    /** @brief XBOT joint velocities */
    Eigen::VectorXd joint_velocities_xbot_;
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
    /** @brief Real time publisher - desired joint states */
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> ci_joint_states_rt_pub_;
    /** @brief Real time publisher - contact forces */
    std::shared_ptr<realtime_tools::RealtimePublisher<wb_controller::ContactForces>> contact_forces_pub_;
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
    /** @brief ROS dynamic reconfigure */
    dynamic_reconfigure::Server<wb_controller::controllerConfig>* server_;
    /** @brief ROS dynamic reconfigure config struct */
    controllerConfig default_config_;
    /** @brief IMU Accelerometer */
    Eigen::Vector3d imu_accelerometer_;
    /** @brief IMU Gyroscope */
    Eigen::Vector3d imu_gyroscope_;
    /** @brief IMU Gyroscope filtered */
    Eigen::Vector3d imu_gyroscope_filt_;
    /** @brief IMU Orientation */
    Eigen::Quaterniond imu_orientation_;
    /** @brief Reference for the waist RPY */
    Eigen::Vector3d des_base_rpy_;
    /** @brief Homing position, loaded from the srdf file */
    Eigen::VectorXd qhome_;
    /** @brief Stance joints position */
    Eigen::VectorXd qstance_;
    /** @brief Swing joints position */
    Eigen::VectorXd qswing_;
    /** @brief Min joints position */
    Eigen::VectorXd qmin_;
    /** @brief Max joints position */
    Eigen::VectorXd qmax_;
    /** @brief Ground reaction forces generated by the solver  */
    Eigen::VectorXd des_contact_forces_;
    /** @brief Feet names */
    std::vector<std::string> feet_names_;
    /** @brief Arm tip name */
    std::string arm_tip_name_;
    /** @brief Hips names */
    std::vector<std::string> hips_names_;
    /** @brief Thread for the odometry publisher */
    std::shared_ptr<std::thread> odom_publisher_thread_;
    /** @brief Thread for the rviz publisher */
    std::shared_ptr<std::thread> rviz_publisher_thread_;
    /** @brief Gait generator */
    wb_controller::GaitGenerator::Ptr gait_generator_;
    /** @brief Ros node handle */
    ros::NodeHandle nh_;
    /** @brief Joy handler */
    JoyHandler::Ptr joy_handler_;
    /** @brief Keyboard handler */
    TwistHandler::Ptr keyboard_handler_;
    /** @brief Command interface */
    FootholdsPlanner::Ptr cmds_;
    /** @brief State estimator */
    StateEstimator::Ptr state_estimator_;
    /** @brief pid scale, range between 0 and 1. 0 the pid is deactivated, 1 the pid is providing full torque */
    std::atomic<double> pid_scale_;
    /** @brief qdot_filter */
    XBot::Utils::SecondOrderFilter<Eigen::VectorXd> qdot_filter_;
    /** @brief imu_gyroscope_filter */
    XBot::Utils::SecondOrderFilter<Eigen::Vector3d> imu_gyroscope_filter_;
     /** @brief cutoff_hz_ */
    std::atomic<double> cutoff_hz_gyro_;
    /** @brief cutoff_hz_ */
    std::atomic<double> cutoff_hz_qdot_;
    /** @brief True if the solver istance has been created */
    bool solver_created_;
    /** @brief True if the solver is started */
    std::atomic<bool> solver_started_;
    /** @brief True if the initialization phase is done */
    std::atomic<bool> init_done_;
    /** @brief True if the pid control is active */
    std::atomic<bool> pid_active_;
    /** @brief True if the haptic contact loop is active */
    std::atomic<bool> haptic_contact_loop_active_;
    /** @brief True if the control of the base height is active */
    std::atomic<bool> base_height_control_active_;
    /** @brief True if the inertia compensation is active */
    std::atomic<bool> inertia_compensation_active_;
    /** @brief True if the controller is stopping */
    std::atomic<bool> stopping_;


    /** @brief Support temporary Affine3d */
    Eigen::Affine3d tmp_affine3d_;
    /** @brief Support temporary Vector3d */
    Eigen::Vector3d tmp_vector3d_;
    /** @brief Support temporary Matrix3d */
    Eigen::Matrix3d tmp_matrix3d_;

    // FIXME to be moved
    Eigen::MatrixXd J_;
    Eigen::MatrixXd J_foot_;
    Eigen::Vector3d x_err_;
    std::atomic<double> clik_gain_;

    // FIXME to be moved
    Eigen::Matrix3d Kp_swing_leg_, Kd_swing_leg_, Kp_stance_leg_, Kd_stance_leg_;
    Eigen::Matrix6d Kp_waist_, Kd_waist_;
    Eigen::MatrixXd M_, Mi_, Kp_postural_, Kd_postural_;


    /**
         * @brief thread body for the odometry publisher
         */
    void odomPublisher();

    /**
         * @brief update the joints state
         */
    void readJoints();

    /**
         * @brief update the imu reading from the imu interface
         */
    void readImu();

    /**
         * @brief check if the joints position are between a max and min value, saturate the value if the limits are violated
         * @param q input vector to check
         * @param qmin min values
         * @param qmax max values
         * @return true is the limits are violated
         */
    bool jointLimitsCheck(Eigen::VectorXd& q, const Eigen::VectorXd& qmin,  const Eigen::VectorXd& qmax);

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
