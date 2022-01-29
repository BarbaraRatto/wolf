#ifndef WOLF_CONTROLLER_H
#define WOLF_CONTROLLER_H

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
// PluginLib
#include <pluginlib/class_list_macros.hpp>
// ROS control
#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
// STD
#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
// WoLF
#include <wolf_controller/quadruped_robot.h>
#include <wolf_controller/gait_generator.h>
#include <wolf_controller/impedance.h>
#include <wolf_controller/footholds_planner.h>
#include <wolf_controller/com_planner.h>
#include <wolf_controller/state_estimator.h>
#include <wolf_controller/terrain_estimator.h>
#include <wolf_controller/id_problem.h>
#include <wolf_controller/ros_wrappers/interface.h>
#include <wolf_controller/devices/interface.h>
#include <wolf_hardware_interface/ground_truth_interface.h>
#include <wolf_hardware_interface/contact_switch_sensor_interface.h>

// Eigen
#include <Eigen/Geometry>

namespace wolf_controller
{

class Controller : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface, // Mandatory interface
                                                                         hardware_interface::ImuSensorInterface, // Mandatory interface
                                                                         hardware_interface::GroundTruthInterface, // Optional interface
                                                                         hardware_interface::ContactSwitchSensorInterface> // Optional interface
{
public:

     enum posture_t {UP=0,DOWN};
     enum mode_t {WALKING=0,MANIPULATION,RESET};

     const std::string CLASS_NAME = "Controller";

    /**
     * @brief Shared pointer to Controller
     */
    typedef std::shared_ptr<Controller> Ptr;

    /**
     * @brief Weak pointer to Controller
     */
    typedef std::weak_ptr<Controller> WeakPtr;

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
         * @param ros::NodeHandle& Controller node handle
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
         * @brief Set the duty factor for the feet
         * @param const double duty_factor
         */
    bool setDutyFactor(const double& duty_factor);

    /**
         * @brief Set the mu value for the friction cones
         * @param mu [0,1]
         */
    bool setFrictionConesMu(const double &mu);

    /**
         * @brief Set the swing frequency
         * @param const double& swing_frequency
         */
    bool setSwingFrequency(const double& swing_frequency);

    /**
         * @brief set cutoff frequency for the qdot filter
         */
    void setCutoffFreqQdot(const double& hz);

    /**
         * @brief set cutoff frequency for the gyroscope filter
         */
    void setCutoffFreqGyro(const double& hz);

    /**
         * @brief Select the control mode to use [WALKING|MANIPULATION]
         */
    bool selectControlMode(const std::string& mode);

    /**
         * @brief Switch between WALKING and MANIPULATION
         */
    void switchControlMode();

    /**
         * @brief select the posture [UP|DOWN]
         */
    bool selectPosture(const std::string& posture);

    /**
         * @brief switch posture between UP and DOWN
         */
    void switchPosture();

    /**
         * @brief stand up
         */
    void standUp(bool stand_up);

    /**
         * @brief emergency stop
         */
    void emergencyStop();

    /**
         * @brief reset base orientation (roll and pitch) and base height
         */
    void resetBase();

    /**
         * @brief Select the gait to use
         */
    bool selectGait(const std::string& gait);

    /**
         * @brief Switch between TROT and CRAWL gait
         */
    void switchGait();

    /**
         * @brief Get desired contact forces
         */
    std::vector<Eigen::Vector6d>& getDesiredContactForces();

    /**
         * @brief Get desired joint efforts
         */
    const Eigen::VectorXd& getDesiredJointEfforts() const;

    /**
         * @brief Get the id problem
         */
    IDProblem* getIDProblem() const;

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

    /**
         * @brief Get the impedance pointer
         */
    Impedance* getImpedance() const;

    /**
         * @brief Get the terrain estimator pointer
         */
    TerrainEstimator* getTerrainEstimator() const;

    /**
         * @brief Get Robot Model
         */
    QuadrupedRobot* getRobotModel() const;


private:

    /** @brief Joint names */
    std::vector<std::string> joint_names_;
    /** @brief Joint states for reading positions, velocities and efforts and writing effort commands */
    std::vector<hardware_interface::JointHandle> joint_states_;
    /** @brief Robot name */
    std::string robot_name_;
    /** @brief Control period */
    double period_;
    /** @brief IMU sensor name */
    std::string imu_name_;
    /** @brief IMU sensors */
    hardware_interface::ImuSensorHandle imu_sensor_;
    /** @brief Ground Thruth */
    hardware_interface::GroundTruthHandle ground_truth_;
    /** @brief Ground Thruth */
    std::map<std::string,hardware_interface::ContactSwitchSensorHandle> contact_sensors_;
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
    /** @brief Desired joint positions */
    Eigen::VectorXd des_joint_positions_;
    /** @brief Desired joint velocities */
    Eigen::VectorXd des_joint_velocities_;
    /** @brief Desired joint efforts computed by the solver */
    Eigen::VectorXd des_joint_efforts_solver_;
    /** @brief Desired joint efforts computed by the impedance control */
    Eigen::VectorXd des_joint_efforts_impedance_;
    /** @brief Desired joint efforts sent to the hardware interface */
    Eigen::VectorXd des_joint_efforts_;
    /** @brief Xbot robot model */
    QuadrupedRobot::Ptr robot_model_;
    /** @brief Impedance pointer */
    Impedance::Ptr impedance_;
    /** @brief Dynamic problem formulation */
    IDProblem::UniquePtr id_prob_;
    /** @brief Desired contact forces */
    std::vector<Eigen::Vector6d> des_contact_forces_;
    /** @brief IMU Accelerometer */
    Eigen::Vector3d imu_accelerometer_;
    /** @brief IMU Gyroscope */
    Eigen::Vector3d imu_gyroscope_;
    /** @brief IMU Gyroscope filtered */
    Eigen::Vector3d imu_gyroscope_filt_;
    /** @brief IMU Orientation */
    Eigen::Quaterniond imu_orientation_;
    /** @brief Ground Truth Orientation */
    Eigen::Quaterniond ground_truth_orientation_;
    /** @brief Thread for the odometry publisher */
    std::shared_ptr<std::thread> odom_publisher_thread_;
    /** @brief Gait generator */
    GaitGenerator::Ptr gait_generator_;
    /** @brief Terrain Estimator */
    TerrainEstimator::Ptr terrain_estimator_;
    /** @brief Ros node handle */
    ros::NodeHandle nh_;
    /** @brief Device handler */
    DeviceHandlerInterface::Ptr device_handler_;
    /** @brief Foot holds Planner */
    FootholdsPlanner::Ptr foot_holds_planner_;
    /** @brief CoM Planner */
    ComPlanner::Ptr com_planner_;
    /** @brief State estimator */
    StateEstimator::Ptr state_estimator_;
    /** @brief Manage the ros interfacing */
    RosWrapperInterface::Ptr ros_wrapper_;
    /** @brief qdot_filter */
    XBot::Utils::SecondOrderFilter<Eigen::VectorXd> qdot_filter_;
    /** @brief imu_gyroscope_filter */
    XBot::Utils::SecondOrderFilter<Eigen::Vector3d> imu_gyroscope_filter_;
    /** @brief True if the controller uses the external contact sensors */
    bool use_contact_sensors_;
    /** @brief True if the controller is stopping */
    std::atomic<bool> stopping_;

    /** @brief Support temporary Affine3d */
    Eigen::Affine3d tmp_affine3d_;
    /** @brief Support temporary Vector3d */
    Eigen::Vector3d tmp_vector3d_;
    /** @brief Support temporary Vector3d */
    Eigen::Vector3d tmp_vector3d_1_;
    /** @brief Support temporary Vector3d */
    Eigen::Vector3d tmp_vector3d_2_;
    /** @brief Support temporary Matrix3d */
    Eigen::Matrix3d tmp_matrix3d_;
    /** @brief Support temporary double */
    double tmp_double_;

    /** @brief Counters used for checks */
    Counter::Ptr solver_failures_cnt_;
    Counter::Ptr contact_failures_cnt_;
    std::vector<Counter::Ptr> velocity_lims_failures_cnt_;

    /** @brief Ramps */
    Ramp::Ptr ramp_stand_up_;
    Ramp::Ptr ramp_stand_down_;
    Ramp::Ptr ramp_impedance_;

    /** @brief state machine support variables */
    unsigned int mode_;
    unsigned int posture_;
    double stand_down_starting_height_;
    double desired_height_;
    double current_height_;
    double previous_height_;
    Eigen::Vector3d current_rpy_;

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
         * @brief update the state estimator
         * @param dt control period
         */
    void updateStateEstimator(const double& dt);

    /**
         * @brief update the state machine
         * @param dt control period
         */
    void updateStateMachine(const double &dt);

    /**
         * @brief initialize the controller's components
         */
    void init();

    /**
         * @brief perform an execution step with the solver
         * @param dt control period
         * @return false if the solver failed
         */
    bool updateSolver(const double &dt);

    /**
         * @brief perform an execution step with the impedance
         * @param des_joint_positions
         * @param des_joint_velocities
         */
    void updateImpedance(const Eigen::VectorXd& des_joint_positions, const Eigen::VectorXd& des_joint_velocities);

    /**
         * @brief perform an execution step with the controller's components
         * such as foot holds planner, com planner and terrain estimator
         * @param dt control period
         */
    void updateComponents(const double &dt);

    /**
         * @brief update base references
         */
    void updateBaseReferences(const Eigen::Vector3d& com_pos_ref,
                              const Eigen::Vector3d& com_vel_ref,
                              const Eigen::Matrix3d& orientation_ref);

    /**
         * @brief perform various safety checks
         * @return false if something failed
         */
    bool performSafetyChecks();

};


PLUGINLIB_EXPORT_CLASS(wolf_controller::Controller, controller_interface::ControllerBase);

} //@namespace wolf_controller

#endif //wolf_controller_H
