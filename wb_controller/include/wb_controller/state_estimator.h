#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/floating_base_estimation/qp_estimation.h>
#include <wb_controller/locomotion.h>

namespace wb_controller
{

class StateEstimator {

public:

    /**
     * @brief Shared pointer to StateEstimator
     */
    typedef std::shared_ptr<StateEstimator> Ptr;

    /**
     * @brief Shared pointer to const StateEstimator
     */
    typedef std::shared_ptr<const StateEstimator> ConstPtr;

    enum estimation_t {IMU_ORIENTATION=0,IMU_GYROSCOPE};

    StateEstimator(GaitGenerator::Ptr gait_generator, XBot::ModelInterface::Ptr xbot_model);

    //~StateEstimator()

    void update(const double& period);

    void setJointPosition(const Eigen::VectorXd& joint_positions);

    void setJointVelocity(const Eigen::VectorXd& joint_velocities);

    void setJointEffort(const Eigen::VectorXd& joint_efforts);

    void setImuOrientation(const Eigen::Quaterniond& imu_orientation);

    void setImuGyroscope(const Eigen::Vector3d& imu_gyroscope);

    void setContactState(const std::string& foot_name, const bool& contact_state);

    const Eigen::Affine3d& getFloatingBasePose() const;

    const Eigen::Vector3d& getFloatingBasePosition() const;

    const Eigen::Vector3d& getFloatingBaseOrientationRPY() const;

private:

    /** @brief Joint positions */
    Eigen::VectorXd joint_positions_;
    /** @brief Joint velocities */
    Eigen::VectorXd joint_velocities_;
    /** @brief Joint efforts */
    Eigen::VectorXd joint_efforts_;
    /** @brief IMU Gyroscope */
    Eigen::Vector3d imu_gyroscope_;
    /** @brief IMU Orientation */
    Eigen::Quaterniond imu_orientation_;
    /** @brief Floating base pose w.r.t world */
    Eigen::Affine3d floating_base_pose_;
    /** @brief Floating base position (x,y,z) w.r.t world */
    Eigen::Vector3d floating_base_position_;
    /** @brief Floating base velocity w.r.t world */
    Eigen::Vector6d floating_base_velocity_;
    /** @brief Floating base velocity, computed by the QP */
    Eigen::VectorXd floating_base_velocity_qp_;
    /** @brief Floating base orientation w.r.t the world frame, computed by the state estimator (RPY) */
    Eigen::Vector3d base_rpy_;
    /** @brief Align the imu frame (trunk) to the world */
    bool imu_reset_done_;

    Eigen::Matrix3d tmp_matrix3d_;

    Eigen::Affine3d tmp_affine3d_;

    XBot::ModelInterface::Ptr xbot_model_;

    GaitGenerator::Ptr gait_generator_;

    std::atomic<unsigned int> estimation_;

    /** @brief Base estimation */
    OpenSoT::floating_base_estimation::qp_estimation::Ptr qp_estimation_;

};


} // namespace


#endif
