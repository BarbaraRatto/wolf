#ifndef WB_CONTROLLER_H
#define WB_CONTROLLER_H

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
#include <wolf_hardware_interface/ground_truth_interface.h>
#include <wolf_hardware_interface/contact_switch_sensor_interface.h>
// STD
#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
// Controller
#include <wb_controller/quadruped_robot.h>
#include <wb_controller/gait_generator.h>
#include <wb_controller/legs_kinematics.h>
#include <wb_controller/legs_impedance.h>
#include <wb_controller/footholds_planner.h>
#include <wb_controller/com_planner.h>
#include <wb_controller/state_estimator.h>
#include <wb_controller/terrain_estimator.h>
#include <wb_controller/id_problem.h>
#include <wb_controller/ros_wrappers/interface.h>
#include <wb_controller/devices/interface.h>

// Eigen
#include <Eigen/Geometry>

namespace wb_controller
{

class Controller : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                                         hardware_interface::ImuSensorInterface,
                                                                         hardware_interface::GroundTruthInterface,
                                                                         hardware_interface::ContactSwitchSensorInterface>
{
public:

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
         * @brief Start/Stop solver integration
         */
    void toggleSolver();

    /**
         * @brief Start/Stop solver integration
         */
    void startSolver(const bool& start);

    /**
         * @brief get the flag value for the solver
         */
    bool isSolverActive() const;

    /**
         * @brief Start/Stop the base height control
         */
    void toggleBaseHeightControl();

    /**
         * @brief Start/Stop the inertia compensation at the leg level, useful if the robot has very low inertia at the knee joints
         */
    void toggleInertiaCompensation();

    /**
         * @brief Start/Stop the inertia compensation at the leg level, useful if the robot has very low inertia at the knee joints
         */
    void startInertiaCompensation(const bool& start);

    /**
         * @brief get the flag value for the inertia compensation
         */
    bool isInertiaCompensationActive() const;

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
         * @brief Get the legs kinematics pointer
         */
    LegsKinematics* getLegsKinematics() const;

    /**
         * @brief Get the legs impedance pointer
         */
    LegsImpedance* getLegsImpedance() const;

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
    /** @brief Desired joint efforts sent to the hardware interface */
    Eigen::VectorXd des_joint_efforts_;
    /** @brief Desired joint efforts computed by the solver */
    Eigen::VectorXd des_joint_efforts_solver_;
    /** @brief Desired joint efforts computed by the PIDs */
    Eigen::VectorXd des_joint_efforts_pids_;
    /** @brief Xbot robot model */
    QuadrupedRobot::Ptr robot_model_;
    /** @brief Dynamic problem formulation */
    IDProblem::UniquePtr id_prob_;
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
    /** @brief Desired contact forces */
    std::vector<Eigen::Vector6d> des_contact_forces_;
    /** @brief Vector containing the pids for the joints */
    std::vector<control_toolbox::Pid> pids_;
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
    /** @brief Reference for the waist RPY */
    Eigen::Vector3d des_base_rpy_;
    /** @brief Thread for the odometry publisher */
    std::shared_ptr<std::thread> odom_publisher_thread_;
    /** @brief Gait generator */
    GaitGenerator::Ptr gait_generator_;
    /** @brief Terrain Estimator */
    TerrainEstimator::Ptr terrain_estimator_;
    /** @brief Legs Kinematics */
    LegsKinematics::Ptr legs_kinematics_;
    /** @brief LegsImpedance */
    LegsImpedance::Ptr legs_impedance_;
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
    /** @brief pid scale, range between 0 and 1. 0 the pid is deactivated, 1 the pid is providing full torque */
    std::atomic<double> pid_scale_;
    /** @brief qdot_filter */
    XBot::Utils::SecondOrderFilter<Eigen::VectorXd> qdot_filter_;
    /** @brief imu_gyroscope_filter */
    XBot::Utils::SecondOrderFilter<Eigen::Vector3d> imu_gyroscope_filter_;
    /** @brief True if the controller uses the external contact sensors */
    bool use_contact_sensors_;

    /** @brief True if the solver is started */
    std::atomic<bool> solver_active_;
    /** @brief True if the initialization phase is done */
    std::atomic<bool> init_done_;
    /** @brief True if the pid control is active */
    std::atomic<bool> pid_active_;
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

    /** @brief Counters used for checks */
    Counter::Ptr solver_failures_cnt_;
    Counter::Ptr contact_failures_cnt_;
    Counter::Ptr velocity_lims_failures_cnt_;

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
         */
    void updateStateEstimator(const double& dt);

};


PLUGINLIB_EXPORT_CLASS(wb_controller::Controller, controller_interface::ControllerBase);

} //@namespace wb_controller

#endif //WB_CONTROLLER_H
