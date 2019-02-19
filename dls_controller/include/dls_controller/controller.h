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
// Hardware interface
#include <dls_hardware_interface/joint_command_adv_interface.h>
// Ros control
#include <controller_interface/controller.h>
// ADVR
#include <cartesian_interface/open_sot/OpenSotImpl.h>
#include <XBotCoreModel/XBotCoreModel.h>
// STD
#include <atomic>

namespace dls_controller
{

class Controller : public controller_interface::Controller<hardware_interface::JointCommandAdvInterface>
{
public:
    /** @brief Constructor function */
    Controller();

    /** @brief Destructor function */
    ~Controller();

    /**
         * @brief Initializes sample controller
         * @param hardware_interface::JointCommandAdvInterface* hardware interface
         * @param ros::NodeHandle& Root node handle
         * @param ros::NodeHandle& Supervisor node handle
         */
    bool init(hardware_interface::JointCommandAdvInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

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

private:

    /** @brief number of joints */
    unsigned int num_joints_;
    /** @brief joint names */
    std::vector<std::string> joint_names_;
    /** @brief joint states for input and output */
    std::vector<hardware_interface::JointCommandAdvHandle> joint_states_;
    /** @brief joint positions */
    Eigen::VectorXd joint_positions_;
    /** @brief joint velocities */
    Eigen::VectorXd joint_velocities_;
    /** @brief joint accellerations */
    Eigen::VectorXd joint_accellerations_;
    /** @brief joint efforts */
    Eigen::VectorXd joint_efforts_;
    /** @brief desired joint positions */
    Eigen::VectorXd des_joint_positions_;
    /** @brief desired joint velocities */
    Eigen::VectorXd des_joint_velocities_;
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
    /** @brief P value for the joints PID controller */
    std::vector<double> desired_joint_p_gain_;
    /** @brief I value for the joints PID controller */
    std::vector<double> desired_joint_i_gain_;
    /** @brief D value for the joints PID controller */
    std::vector<double> desired_joint_d_gain_;
    /** @brief Integrate the solver solution and apply it to the desired joints state */
    std::atomic<bool> solver_started_;
    /** @brief ROS service server */
    ros::ServiceServer ss_;

    void odomPublisher();
};


PLUGINLIB_EXPORT_CLASS(dls_controller::Controller, controller_interface::ControllerBase);

} //@namespace dls_controller

#endif //DLS_CONTROLLER_H
