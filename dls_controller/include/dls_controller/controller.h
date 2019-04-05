#ifndef DLS_CONTROLLER_H
#define DLS_CONTROLLER_H

// Ros
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <dls_controller/DlsControllerServices.h>
// PluginLib
#include <pluginlib/class_list_macros.hpp>
// Ros control
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
// Hardware interfaces
#include <dls_hardware_interface/joint_command_adv_interface.h> // custom hw
#include <hardware_interface/imu_sensor_interface.h>
// ADVR
#include <cartesian_interface/open_sot/OpenSotImpl.h>
#include <XBotCoreModel/XBotCoreModel.h>
#include <dls_controller/ForceOptimization.h>
// STD
#include <atomic>
#include <thread>
#include <chrono>

namespace dls_controller
{

class Controller : public controller_interface::MultiInterfaceController<hardware_interface::JointCommandAdvInterface,
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
         * @brief Set the com reference for the solver
         * @param geometry_msgs::Point::ConstPtr& msg
         */
    void setComReference(const geometry_msgs::Point::ConstPtr& msg);

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
         * @brief Start/Stop gravity compensation
         */
    void toggleGravityCompensation();

    /**
         * @brief Start/Stop the PIDs
         */
    void togglePid();

private:

    /** @brief Number of joints */
    unsigned int num_joints_;
    /** @brief Joint names */
    std::vector<std::string> joint_names_;
    /** @brief Imu sensor names */
    std::vector<std::string> imu_names_;
    /** @brief Joint states for input and output */
    std::vector<hardware_interface::JointCommandAdvHandle> joint_states_;
    /** @brief IMU sensors */
    std::vector<hardware_interface::ImuSensorHandle> imu_sensors_;
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
    /** @brief Xbot robot model */
    XBot::ModelInterface::Ptr xbot_model_;
    /** @brief Cartesian Interface Pointer */
    XBot::Cartesian::CartesianInterfaceImpl::Ptr ci_;
    /** @brief Real time publisher */
    realtime_tools::RealtimePublisher<sensor_msgs::JointState>* realtime_pub_;
    /** @brief Ros subscriber for the com position reference */
    ros::Subscriber com_ref_sub_;
    /** @brief Ros publisher for the com position */
    ros::Publisher com_pub_;
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
    /** @brief Actual com position w.r.t world frame */
    Eigen::Vector3d com_position_;
    /** @brief Desired com position w.r.t world frame */
    Eigen::Vector3d des_com_position_;
    /** @brief Integrate the solver solution and apply it to the desired joints state */
    std::atomic<bool> solver_started_;
    /** @brief Activate gravity compensation */
    std::atomic<bool> gravity_compensation_;
    /** @brief Activate pid gains */
    std::atomic<bool> pid_active_;
    /** @brief Variable used to signal that the controller is stopping */
    std::atomic<bool> stopping_;
    /** @brief ROS service server */
    ros::ServiceServer ss_;
    /** @brief Force Optimization Pointer */
    OpenSoT::utils::ForceOptimization::Ptr fo_;
    /** @brief Accelerometer */
    Eigen::Vector3d imu_accelerometer_;
    /** @brief Gyroscope */
    Eigen::Vector3d imu_gyroscope_;
    /** @brief Orientation */
    Eigen::Quaterniond imu_orientation_;
    /** @brief Homing position, loaded from the srdf file */
    Eigen::VectorXd qhome_;

    std::shared_ptr<std::thread> odom_publisher_thread_;

    void odomPublisher();

    void readJoints();

    void readImu();
};


PLUGINLIB_EXPORT_CLASS(dls_controller::Controller, controller_interface::ControllerBase);

} //@namespace dls_controller

#endif //DLS_CONTROLLER_H
