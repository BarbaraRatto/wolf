/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>
#include <OpenSoT/floating_base_estimation/qp_estimation.h>
#include <cartesian_interface/utils/estimation/ForceEstimation.h>
#include <wolf_controller/gait_generator.h>
#include <wolf_controller/quadruped_robot.h>

namespace wolf_controller
{

class StateEstimator {

public:

    const std::string CLASS_NAME = "StateEstimator";

    /**
     * @brief Shared pointer to StateEstimator
     */
    typedef std::shared_ptr<StateEstimator> Ptr;

    /**
     * @brief Shared pointer to const StateEstimator
     */
    typedef std::shared_ptr<const StateEstimator> ConstPtr;

    enum estimation_t {NONE=0,IMU_MAGNETOMETER,IMU_GYROSCOPE,GROUND_TRUTH,ESTIMATED_Z};

    StateEstimator(GaitGenerator::Ptr gait_generator, QuadrupedRobot::Ptr robot_model);

    //~StateEstimator()

    void update(const double& period);

    void setEstimationType(const std::string& position_t, const std::string& orientation_t);

    void setPositionEstimationType(const std::string& position_t);

    void setOrientationEstimationType(const std::string& orientation_t);

    void setJointPosition(const Eigen::VectorXd& joint_positions);

    void setJointVelocity(const Eigen::VectorXd& joint_velocities);

    void setJointEffort(const Eigen::VectorXd& joint_efforts);

    void setTerrainNormal(const Eigen::Vector3d& terrain_normal);

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

    std::string getPositionEstimationType();

    std::string getOrientationEstimationType();

    void setContactForces(const std::string &name, const Eigen::Vector3d &force);

    const std::map<std::string,Eigen::Vector3d>& getContactForces() const;

    const std::map<std::string,bool>& getContacts() const;

    const std::map<std::string, Eigen::Vector3d>& getContactPositionInWorld() const;

    const std::map<std::string, Eigen::Vector3d>& getContactPositionInBase() const;

    const Eigen::Vector3d& getGroundTruthBasePosition() const;

    const Eigen::Quaterniond& getGroundTruthBaseOrientation() const;

    const Eigen::Vector3d& getGroundTruthBaseLinearVelocity() const;

    const Eigen::Vector3d& getGroundTruthBaseAngularVelocity() const;

    const double& getEstimatedBaseHeight() const;

    void startContactComputation();

    void stopContactComputation();

    void toggleHapticContactLoop();

    void startHapticContactLoop();

    void stopHapticContactLoop();

    void resetGyroscopeIntegration();

private:

    void setEstimationType(estimation_t position_t, estimation_t orientation_t);

    void setPositionEstimationType(estimation_t position_t);

    void setOrientationEstimationType(estimation_t orientation_t);

    void updateFloatingBase(const double& period);

    void updateContactState();

    double estimateZ();

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
    /** @brief Ground Truth position */
    Eigen::Vector3d gt_position_;
    /** @brief Ground Truth orientation */
    Eigen::Quaterniond gt_orientation_;
    /** @brief Ground Truth linear velocity */
    Eigen::Vector3d gt_linear_velocity_;
    /** @brief IMU Ground Truth angular velocity */
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
    std::map<std::string,XBot::ForceTorqueSensor::ConstPtr> force_torque_sensors_;
    /** @brief Contact positions w.r.t world */
    std::map<std::string,Eigen::Vector3d> world_X_contact_;
    /** @brief Contact positions w.r.t base */
    std::map<std::string,Eigen::Vector3d> base_X_contact_;
    /** @brief GRF contacts */
    std::map<std::string,bool> contacts_;
    /** @brief GRF contact forces */
    std::map<std::string,Eigen::Vector3d> contact_forces_;
    /** @brief Contact force threshold, this is a normalized value. The actual contact force get compared to this value and if greater equal the contact
    is consired true */
    std::atomic<double> contact_force_th_;

    std::atomic<bool> contact_computation_active_;

    std::atomic<bool> haptic_contact_loop_active_;

    Eigen::Vector3d terrain_normal_;

    Eigen::Vector3d terrain_central_point_;

    Eigen::Matrix3d mapRPYderivativesToOmega_;

    Eigen::Matrix3d world_R_base_;

    Eigen::Matrix3d raw_world_R_base_;

    Eigen::Vector3d raw_base_rpy_;

    /** @brief Reset the gyroscope integration */
    bool reset_gyro_integration_done_;

    Eigen::Matrix3d tmp_matrix3d_;

    Eigen::Affine3d tmp_affine3d_;

    Eigen::Vector3d tmp_vector3d_;

    QuadrupedRobot::Ptr robot_model_;

    GaitGenerator::Ptr gait_generator_;

    estimation_t estimation_orientation_;

    estimation_t estimation_position_;

    std::atomic<bool> use_external_forces_;

    /** @brief Base estimation */
    OpenSoT::floating_base_estimation::qp_estimation::Ptr qp_estimation_;

    /** @brief Base estimated height wrt the feet */
    double estimated_z_;

};


} // namespace


#endif
