#ifndef DLS_CONTROLLER_H
#define DLS_CONTROLLER_H

// Ros
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16MultiArray.h>
#include <dls_controller/DlsControllerServices.h>
#include <dls_controller/TaskPoses.h>
#include <realtime_tools/realtime_buffer.h>
#include <dynamic_reconfigure/server.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <dls_controller/DlsControllerConfig.h>
// PluginLib
#include <pluginlib/class_list_macros.hpp>
// Ros control
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
// Hardware interfaces
#include <dls_hardware_interface/joint_command_adv_interface.h>
#include <dls_hardware_interface/ground_truth_interface.h>
#include <dls_hardware_interface/contact_switch_sensor_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
// ADVR
#include <cartesian_interface/open_sot/OpenSotImpl.h>
#include <XBotCoreModel/XBotCoreModel.h>
#include <OpenSoT/floating_base_estimation/qp_estimation.h>
#include <dls_controller/IDProblem.h>
// STD
#include <atomic>
#include <thread>
#include <chrono>
// Controller
#include <dls_controller/publishers.h>
#include <dls_controller/locomotion.h>
#include <dls_controller/joy.h>

#include <Eigen/Geometry>

namespace dls_controller
{
// FIXME move to a class with publishers and subscribers
typedef std::pair<Eigen::Affine3d,Eigen::Vector6d> Task;
typedef std::map<std::string,Task> TaskPosesMap;
typedef std::map<std::string,std::string> BaseFramesMap;

class Controller : public controller_interface::MultiInterfaceController<hardware_interface::JointCommandAdvInterface,
        hardware_interface::ImuSensorInterface,
        hardware_interface::GroundTruthInterface,
        hardware_interface::ContactSwitchSensorInterface>
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
         * @brief Set the desired reference for the solver's tasks
         * @param dls_controller::TaskPoses::ConstPtr& msg
         */
    void setTasksDesired(const dls_controller::TaskPoses::ConstPtr& msg);

    /**
         * @brief Ros dynamic reconfigure callback
         */
    void dynamicReconfigureCallback(dls_controller::DlsControllerConfig &config, uint32_t level);

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
         * @brief Start/Stop the relative tasks
         */
    void toggleRelativeTasks();

    /**
         * @brief Set the lambda gains of the tasks
         * @param const std::string& task_name
         * @param const double lambda_value
         */
    bool setLambda(const std::string& task_name, const double& lambda_value);

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
    /** @brief Imu sensor names */
    std::vector<std::string> imu_names_;
    /** @brief State estimator names */
    std::vector<std::string> state_estimator_names_;
    /** @brief State estimator names */
    std::vector<std::string> contact_sensor_names_;
    /** @brief Joint states for input and output */
    std::vector<hardware_interface::JointCommandAdvHandle> joint_states_;
    /** @brief IMU sensors */
    std::vector<hardware_interface::ImuSensorHandle> imu_sensors_;
    /** @brief State Estimation */
    std::vector<hardware_interface::GroundTruthHandle> state_estimators_; // FIXME We should use a state estimator handle no matter if the robot is simulated or no
    /** @brief Contact sensors */
    std::vector<hardware_interface::ContactSwitchSensorHandle> contact_sensors_;
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
    /** @brief Desired joint efforts */
    Eigen::VectorXd des_joint_efforts_;
    /** @brief Solver's solution (i.e. efforts) */
    Eigen::VectorXd x_;
    /** @brief Xbot robot model */
    XBot::ModelInterface::Ptr xbot_model_;
    /** @brief Dynamic problem formulation */
    OpenSoT::IDProblem::Ptr id_prob_;
     /** @brief Base estimation */
    OpenSoT::floating_base_estimation::qp_estimation::Ptr qp_estimation_;
    /** @brief Real time publisher - desired joint states */
    realtime_tools::RealtimePublisher<sensor_msgs::JointState>* ci_joint_states_rt_pub_;
    /** @brief Real time publisher - QP */
    realtime_tools::RealtimePublisher<nav_msgs::Odometry>* state_estimation_qp_rt_pub_;
    /** @brief Real time publisher - estimated pose */
    realtime_tools::RealtimePublisher<nav_msgs::Odometry>* state_estimation_rt_pub_;
    /** @brief Real time publisher - actual tasks pose */
    realtime_tools::RealtimePublisher<dls_controller::TaskPoses>* tasks_actual_pose_rt_pub_;
    /** @brief Real time publisher - desired tasks pose */
    realtime_tools::RealtimePublisher<dls_controller::TaskPoses>* tasks_desired_pose_rt_pub_;
    /** @brief Real time publisher - contacts */
    realtime_tools::RealtimePublisher<std_msgs::Int16MultiArray>* contacts_rt_pub_;
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
    /** @brief Actual task poses and velocities */
    TaskPosesMap task_poses_;
    /** @brief Desired task poses and velocities */
    TaskPosesMap desired_task_poses_;
    /** @brief Desired task poses */
    BaseFramesMap base_frames_;
    /** @brief Actual com position w.r.t world frame */
    Eigen::Vector3d com_position_;
    /** @brief Desired com position w.r.t world frame */
    Eigen::Vector3d  des_com_position_;
    /** @brief  RT buffer for the desired poses of the id tasks */
    //realtime_tools::RealtimeBuffer<TaskPosesMap> desired_tasks_pose_;
    /** @brief Integrate the solver solution and apply it to the desired joints state */
    std::atomic<bool> solver_started_;
    /** @brief Activate pid gains */
    std::atomic<bool> pid_active_;
    /** @brief Activate tracking */
    std::atomic<bool> tracking_active_;
    /** @brief Activate relative tasks */
    std::atomic<bool> relative_tasks_active_;
    /** @brief Variable used to signal that the controller is stopping */
    std::atomic<bool> stopping_;
    /** @brief ROS dynamic reconfigure */
    dynamic_reconfigure::Server<dls_controller::DlsControllerConfig>* server_;
    /** @brief ROS dynamic reconfigure config struct */
    DlsControllerConfig default_config_;
    /** @brief IMU Accelerometer */
    Eigen::Vector3d imu_accelerometer_;
    /** @brief IMU Gyroscope */
    Eigen::Vector3d imu_gyroscope_;
    /** @brief IMU Orientation */
    Eigen::Quaterniond imu_orientation_;
    /** @brief Homing position, loaded from the srdf file */
    Eigen::VectorXd qhome_;
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
    /** @brief Floating base accelleration, computed by the state estimator */
    Eigen::Vector6d floating_base_accelleration_;
    /** @brief Floating base pose w.r.t the world frame, computed by the state estimator */
    Eigen::Affine3d floating_base_pose_;    
    /** @brief GRF normals */
    std::vector<Eigen::Vector3d> normals_;
    /** @brief GRF contacts */
    std::vector<bool> contacts_;
    /** @brief GRF contact forces */
    std::vector<Eigen::Vector3d> contact_forces_;
    /** @brief Feet names */
    std::vector<std::string> feet_names_;
    /** @brief Hips names */
    std::vector<std::string> hips_names_;
    /** @brief Thread for the odometry publisher */
    std::shared_ptr<std::thread> odom_publisher_thread_;
    /** @brief Thread for the rviz publisher */
    std::shared_ptr<std::thread> rviz_publisher_thread_;
    /** @brief True if the solver has been resetted */
    bool solver_reset_done_;
    /** @brief Gait generator */
    dls_controller::GaitGenerator::Ptr gait_generator_;
    /** @brief Visual tools */
    std::map<std::string,rviz_visual_tools::RvizVisualToolsPtr> visual_tools_;
    /** @brief Joy handler */
    JoyHandler::Ptr joy_handler_;
    /** @brief Command interface */
    CommandsInterface::Ptr cmds_;
    /** @brief Support temporary Affine3d */
    Eigen::Affine3d tmp_affine3d_;
    /** @brief Support temporary Vector3d */
    Eigen::Vector3d tmp_vector3d_;
    /** @brief Support temporary Matrix3d */
    Eigen::Matrix3d tmp_matrix3d_;

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
         * @brief publish on ROS
         */
    void publish(const ros::Time& time, const ros::Duration& period);

    /**
         * @brief set the initial poses for the gait generator for the specified foot w.r.t to base_frame
         */
    void setInitialPose(const std::string& base_frame, const std::string& contact_name);

    /**
         * @brief set the initial poses for the gait generator for each foot w.r.t to base_frame
         */
    void setInitialPose(const std::string& base_frame);

    /**
         * @brief set the initial poses for the gait generator for each foot w.r.t to the current frame
         */
    void setInitialPose();

    /**
         * @brief set the relative tasks
         */
    void setRelativeTasks();

    /**
         * @brief set the world tasks
         */
    void setWorldTasks();

    /**
         * @brief Update the dynamic reconfigure interface
         */
    void dynamicReconfigureUpdate();
};


PLUGINLIB_EXPORT_CLASS(dls_controller::Controller, controller_interface::ControllerBase);

} //@namespace dls_controller

#endif //DLS_CONTROLLER_H
