#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/floating_base_estimation/qp_estimation.h>
#include <cartesian_interface/utils/estimation/ForceEstimation.h>
#include <wb_controller/gait_generator.h>

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

    enum estimation_t {NONE=0,IMU_MAGNETOMETER,IMU_GYROSCOPE,GROUND_TRUTH,ESTIMATED_Z};

    StateEstimator(GaitGenerator::Ptr gait_generator, XBot::ModelInterface::Ptr xbot_model);

    //~StateEstimator()

    void update(const double& period);

    void setEstimationType(const std::string& position_t, const std::string& orientation_t);

    void setPositionEstimationType(const std::string& position_t);

    void setOrientationEstimationType(const std::string& orientation_t);

    void setJointPosition(const Eigen::VectorXd& joint_positions);

    void setJointVelocity(const Eigen::VectorXd& joint_velocities);

    void setJointEffort(const Eigen::VectorXd& joint_efforts);

    void setImuOrientation(const Eigen::Quaterniond& imu_orientation);

    void setImuGyroscope(const Eigen::Vector3d& imu_gyroscope);

    void setGroundTruthBasePosition(const Eigen::Vector3d& gt_position);

    void setGroundTruthBaseOrientation(const Eigen::Quaterniond& gt_orientation);

    void setGroundTruthBaseLinearVelocity(const Eigen::Vector3d& gt_linear_velocity);

    void setGroundTruthBaseAngularVelocity(const Eigen::Vector3d& gt_angular_velocity);

    void setContactState(const std::string& foot_name, const bool& contact_state);

    void setContactThreshold(const double& th);

    const Eigen::Affine3d& getFloatingBasePose() const;

    const Eigen::Vector3d& getFloatingBasePosition() const;

    const Eigen::Vector3d& getFloatingBaseOrientationRPY() const;

    double getContactThreshold();

    const std::string& getPositionEstimationType();

    const std::string& getOrientationEstimationType();

    const std::vector<Eigen::Vector3d>& getContactForces() const;

    const std::vector<bool>& getContacts() const;

    const std::vector<Eigen::Vector3d>& getFeetPositionInWorld() const;

    const std::vector<Eigen::Vector3d>& getFeetPositionInBase() const;

    const std::vector<Eigen::Affine3d>& getFeetPoseInWorld() const;

    const std::vector<Eigen::Affine3d>& getFeetPoseInBase() const;

    void startContactsEstimation();

    void stopContactsEstimation();

    void toggleHapticContactLoop();

    void resetGyroscopeIntegration();

private:

    void setEstimationType(unsigned int position_t, unsigned int orientation_t);

    void setPositionEstimationType(unsigned int position_t);

    void setOrientationEstimationType(unsigned int orientation_t);

    void updateFloatingBase(const double& period);

    void updateContactState();

    std::map<std::string,unsigned int> estimations_;

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

    Eigen::Vector3d gt_position_;

    Eigen::Quaterniond gt_orientation_;

    Eigen::Vector3d gt_linear_velocity_;

    Eigen::Vector3d gt_angular_velocity_;

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
    /** @brief Contact estimation */
    XBot::Cartesian::Utils::ForceEstimation::Ptr force_estimation_;
    /** @brief Contact estimation */
    std::vector<XBot::ForceTorqueSensor::ConstPtr> force_torque_sensors_;
    /** @brief Feet positions w.r.t base */
    std::vector<Eigen::Vector3d> base_X_foot_;
    /** @brief Feet positions w.r.t world */
    std::vector<Eigen::Vector3d> world_X_foot_;
    /** @brief Feet pose w.r.t base */
    std::vector<Eigen::Affine3d> base_T_foot_;
    /** @brief Feet pose w.r.t world */
    std::vector<Eigen::Affine3d> world_T_foot_;
    /** @brief GRF contacts */
    std::vector<bool> contacts_;
    /** @brief GRF contact forces */
    std::vector<Eigen::Vector3d> contact_forces_;
    /** @brief Contact force threshold, this is a normalized value. The actual contact force get compared to this value and if greater equal the contact
    is consired true */
    std::atomic<double> contact_force_th_;

    std::atomic<bool> contacts_estimation_active_;

    std::atomic<bool> haptic_contact_loop_active_;

    Eigen::Vector3d terrain_normal_;

    Eigen::Matrix3d mapRPYderivativesToOmega_;

    Eigen::Matrix3d base_R_world_;

    Eigen::Matrix3d raw_base_R_world_;

    Eigen::Vector3d raw_base_rpy_;

    /** @brief Reset the gyroscope integration */
    bool reset_gyro_integration_done_;

    Eigen::Vector3d com_;

    Eigen::Matrix3d tmp_matrix3d_;

    Eigen::Affine3d tmp_affine3d_;

    Eigen::Vector3d tmp_vector3d_;

    XBot::ModelInterface::Ptr xbot_model_;

    GaitGenerator::Ptr gait_generator_;

    std::atomic<unsigned int> estimation_orientation_;

    std::atomic<unsigned int> estimation_position_;

    /** @brief Base estimation */
    OpenSoT::floating_base_estimation::qp_estimation::Ptr qp_estimation_;

};


} // namespace


#endif
