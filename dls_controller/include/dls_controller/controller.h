#ifndef DLS_CONTROLLER_H
#define DLS_CONTROLLER_H

// Ros
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <dls_controller/DlsControllerServices.h>
#include <dls_controller/TasksPose.h>
#include <realtime_tools/realtime_buffer.h>
// PluginLib
#include <pluginlib/class_list_macros.hpp>
// Ros control
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
// Hardware interfaces
#include <dls_hardware_interface/joint_command_adv_interface.h> // custom hw
#include <hardware_interface/imu_sensor_interface.h>
#include <dls_hardware_interface/ground_truth_interface.h>
// ADVR
#include <cartesian_interface/open_sot/OpenSotImpl.h>
#include <XBotCoreModel/XBotCoreModel.h>
#include <dls_controller/IDProblem.h>
// STD
#include <atomic>
#include <thread>
#include <chrono>

namespace dls_controller
{

typedef std::map<std::string,Eigen::Affine3d> TasksPoseMap;

class Controller : public controller_interface::MultiInterfaceController<hardware_interface::JointCommandAdvInterface,
        hardware_interface::ImuSensorInterface,
        hardware_interface::GroundTruthInterface>
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
         * @param dls_controller::TasksPose::ConstPtr& msg
         */
    void setTasksDesired(const dls_controller::TasksPose::ConstPtr& msg);

    /**
         * @brief Manage the ros services
         */
    bool servicesManager(dls_controller::DlsControllerServices::Request &req,
                         dls_controller::DlsControllerServices::Response &res);

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

private:

    /** @brief Number of joints */
    unsigned int num_joints_;
    /** @brief Joint names */
    std::vector<std::string> joint_names_;
    /** @brief Imu sensor names */
    std::vector<std::string> imu_names_;
    /** @brief State estimator names */
    std::vector<std::string> state_estimator_names_;
    /** @brief Joint states for input and output */
    std::vector<hardware_interface::JointCommandAdvHandle> joint_states_;
    /** @brief IMU sensors */
    std::vector<hardware_interface::ImuSensorHandle> imu_sensors_;
    /** @brief State Estimation */
    std::vector<hardware_interface::GroundTruthHandle> state_estimators_; // FIXME We should use a state estimator handle no matter if the robot is simulated or no
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
    /** @brief Desired feet poses: */
    Eigen::Affine3d des_lf_foot_pose_;
    Eigen::Affine3d des_lh_foot_pose_;
    Eigen::Affine3d des_rh_foot_pose_;
    Eigen::Affine3d des_rf_foot_pose_;
    /** @brief Xbot robot model */
    XBot::ModelInterface::Ptr xbot_model_;
    /** @brief Dynamic problem formulation */
    OpenSoT::IDProblem::Ptr id_prob_;
    /** @brief Real time publisher - desired joint states */
    realtime_tools::RealtimePublisher<sensor_msgs::JointState>* ci_joint_states_rt_pub_;
    /** @brief Real time publisher - estimated pose */
    realtime_tools::RealtimePublisher<nav_msgs::Odometry>* state_estimation_rt_pub_;
    /** @brief Real time publisher - tasks pose */
    realtime_tools::RealtimePublisher<dls_controller::TasksPose>* tasks_actual_pose_rt_pub_;
    /** @brief Ros subscriber for the desired tasks reference */
    ros::Subscriber tasks_desired_sub_;
    /** @brief Ros subscriber for the limbs pose reference */
    ros::Subscriber limbs_ref_sub_;
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
    /** @brief Tasks running on the robot and respective actual poses */
    TasksPoseMap tasks_pose_;
    /** @brief Actual com position w.r.t world frame */
    Eigen::Vector3d com_position_;
    /** @brief Desired com position w.r.t world frame */
    Eigen::Vector3d  des_com_position_;
    /** @brief  RT buffer for the desired poses of the id tasks */
    realtime_tools::RealtimeBuffer<TasksPoseMap> desired_tasks_pose_;
    /** @brief Desired limbs pose w.r.t world frame */
    //realtime_tools::RealtimeBuffer<LimbsMap> des_limbs_pose_;
    /** @brief Integrate the solver solution and apply it to the desired joints state */
    std::atomic<bool> solver_started_;
    /** @brief Activate gravity compensation */
    std::atomic<bool> gravity_compensation_;
    /** @brief Activate pid gains */
    std::atomic<bool> pid_active_;
    /** @brief Activate tracking */
    std::atomic<bool> traking_active_;
    /** @brief Variable used to signal that the controller is stopping */
    std::atomic<bool> stopping_;
    /** @brief ROS service server */
    ros::ServiceServer ss_;
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
    /** @brief Floating base orientation w.r.t the world frame, computed by the state estimator */
    Eigen::Quaterniond floating_base_orientation_;
    /** @brief Floating base velocity, computed by the state estimator */
    Eigen::Vector6d floating_base_velocity_;
    /** @brief Floating base accelleration, computed by the state estimator */
    Eigen::Vector6d floating_base_accelleration_;
    /** @brief Floating base pose w.r.t the world frame, computed by the state estimator */
    Eigen::Affine3d floating_base_pose_;

    std::vector<std::string> contact_links_;

    bool solver_reset_done_;

    std::shared_ptr<std::thread> odom_publisher_thread_;

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
         * @brief update floating base state
         */
    void stateEstimation();

    /**
         * @brief update the virtual model
         */
    void updateXBotModel();
};


PLUGINLIB_EXPORT_CLASS(dls_controller::Controller, controller_interface::ControllerBase);

} //@namespace dls_controller

#endif //DLS_CONTROLLER_H
