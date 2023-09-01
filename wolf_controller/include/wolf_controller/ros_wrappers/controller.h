/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef ROS_WRAPPERS_CONTROLLER_H
#define ROS_WRAPPERS_CONTROLLER_H

// ROS
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_srvs/Trigger.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <wolf_controller/common.h>

// Generated
#include <wolf_msgs/ContactForces.h>
#include <wolf_msgs/FootHolds.h>
#include <wolf_msgs/TerrainEstimation.h>
#include <wolf_msgs/FrictionCones.h>
#include <wolf_msgs/CapturePoint.h>
#include <wolf_msgs/ControllerState.h>
#include <wolf_msgs/Float32.h>

// WoLF
#include <wolf_controller/controller.h>
#include <wolf_controller/ros_wrappers/interface.h>

// OCS2
#ifdef OCS2
#include <ocs2_msgs/mpc_observation.h>
#endif

class ControllerRosWrapper : public RosWrapperInterface
{

public:

    const std::string CLASS_NAME = "ControllerRosWrapper";

    typedef std::shared_ptr<ControllerRosWrapper> Ptr;

    ControllerRosWrapper(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh, wolf_controller::Controller* const controller_ptr);

    bool increaseSwingFrequencyCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool decreaseSwingFrequencyCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool setSwingFrequencyCB(wolf_msgs::Float32Request& req, wolf_msgs::Float32Response& res);

    bool setDutyFactorCB(wolf_msgs::Float32Request& req, wolf_msgs::Float32Response& res);

    bool activatePushRecoveryCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool activateStepReflexCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool setStepHeightCB(wolf_msgs::Float32Request& req, wolf_msgs::Float32Response& res);

    bool increaseStepHeightCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool decreaseStepHeightCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool switchControlModeCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool switchGaitCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool switchPostureCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool emergencyStopCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool resetBaseCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool standUpCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool standDownCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    virtual void publish(const ros::Time& time, const ros::Duration& period) override;

protected:

    /** @brief Real time publisher - controller state */
    std::shared_ptr<realtime_tools::RealtimePublisher<wolf_msgs::ControllerState>> controller_state_pub_;
    /** @brief Real time publisher - contact forces */
    std::shared_ptr<realtime_tools::RealtimePublisher<wolf_msgs::ContactForces>> contact_forces_pub_;
    /** @brief Real time publisher - foot holds */
    std::shared_ptr<realtime_tools::RealtimePublisher<wolf_msgs::FootHolds>> foot_holds_pub_;
    /** @brief Real time publisher - terrain estimation */
    std::shared_ptr<realtime_tools::RealtimePublisher<wolf_msgs::TerrainEstimation>> terrain_estimation_pub_;
    /** @brief Real time publisher - friction cones */
    std::shared_ptr<realtime_tools::RealtimePublisher<wolf_msgs::FrictionCones>> friction_cones_pub_;
    /** @brief Real time publisher - capture point */
    std::shared_ptr<realtime_tools::RealtimePublisher<wolf_msgs::CapturePoint>> capture_point_pub_;
    /** @brief Real time publisher - OCS2 */
    #ifdef OCS2
    std::shared_ptr<realtime_tools::RealtimePublisher<ocs2_msgs::mpc_observation>> mpc_observation_pub_;
    #endif
    /** @brief Controller pnt */
    wolf_controller::Controller* controller_;
    /** @brief ROS services */
    ros::ServiceServer switch_control_mode_;
    ros::ServiceServer switch_gait_;
    ros::ServiceServer switch_posture_;
    ros::ServiceServer stand_up_srv_;
    ros::ServiceServer stand_down_srv_;
    ros::ServiceServer emergency_stop_srv_;
    ros::ServiceServer reset_base_srv_;
    ros::ServiceServer increase_step_height_;
    ros::ServiceServer decrease_step_height_;
    ros::ServiceServer set_step_height_;
    ros::ServiceServer activate_push_recovery_;
    ros::ServiceServer activate_step_reflex_;
    ros::ServiceServer set_swing_frequency_;
    ros::ServiceServer set_duty_factor_;
    ros::ServiceServer increase_swing_frequency_;
    ros::ServiceServer decrease_swing_frequency_;

    /** @brief tmp variables */
    Eigen::Vector6d tmp_vector6d_;
    Eigen::Affine3d tmp_affine3d_;
    Eigen::Vector3d tmp_vector3d_;
    Eigen::VectorXd tmp_vectorXd_;

};

#endif // ROS_WRAPPERS_CONTROLLER_H

