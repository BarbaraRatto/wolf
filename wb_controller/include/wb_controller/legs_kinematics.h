#ifndef LEGS_KINEMATICS_H
#define LEGS_KINEMATICS_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <wb_controller/geometry.h>
#include <wb_controller/gait_generator.h>
#include <wb_controller/quadruped_robot.h>
#include <wb_controller/terrain_estimator.h>

namespace wb_controller
{

class LegsKinematics
{

public:

  const std::string CLASS_NAME = "LegsKinematics";

  /**
   * @brief Shared pointer to LegsKinematics
   */
  typedef std::shared_ptr<LegsKinematics> Ptr;

  /**
   * @brief Shared pointer to const LegsKinematics
   */
  typedef std::shared_ptr<const LegsKinematics> ConstPtr;

  LegsKinematics(GaitGenerator::Ptr gait_generator, QuadrupedRobot::Ptr robot_model, TerrainEstimator::Ptr terrain_estimator);

  /**
       * @brief Compute the desired joint positions and velocities
       * @param control period
       * @param current joint positions
       * @return true is the joints position limits are violated
       */
  bool update(const double& period, const Eigen::VectorXd& current_joint_positions);

  void setDesiredBaseHeight(const double& des_base_height);

  void setFeedForwardStanceDot(const Eigen::Vector3d& xdot_stance_ff);

  void setFeedForwardSwingDot(const Eigen::Vector3d& xdot_swing_ff);

  void setAdaptiveDamping(const double& damp_max, const double& determinant_max);

  bool setJointHomePositions(Eigen::VectorXd& qhome);

  const Eigen::VectorXd& getJointHomePositions();

  const Eigen::VectorXd& getDesiredJointPositions();

  const Eigen::VectorXd& getDesiredJointVelocities();

  void setClikGain(const double& clik_gain);

  double getClikGain();

  void toggleBaseHeightControl();

  void activateBaseHeightControl();

  void deactivateBaseHeightControl();

  bool isBaseHeightControlActive();

  void reset();

private:

  QuadrupedRobot::Ptr robot_model_;

  GaitGenerator::Ptr gait_generator_;

  TerrainEstimator::Ptr terrain_estimator_;

  /** @brief True if the control of the base height is active */
  std::atomic<bool> base_height_control_active_;

  std::atomic<double> clik_gain_;

  std::atomic<double> des_base_height_;

  double base_height_;
  double base_height_dot_;

  /** @brief Stance joints position */
  Eigen::VectorXd qstance_;
  /** @brief Swing joints position */
  Eigen::VectorXd qswing_;
  /** @brief Desired joint positions */
  Eigen::VectorXd des_joint_positions_;
  /** @brief Desired joint velocities */
  Eigen::VectorXd des_joint_velocities_;
  /** @brief Homing position */
  Eigen::VectorXd qhome_;
  /** @brief Feet pose w.r.t world */
  Eigen::Affine3d world_T_foot_;
  /** @brief Stance feed forward term in the clik control loop */
  Eigen::Vector3d xdot_stance_ff_;
  /** @brief Swing feed forward term in the clik control loop */
  Eigen::Vector3d xdot_swing_ff_;

  Eigen::MatrixXd J_;
  Eigen::Matrix3d J_foot_;
  Eigen::Matrix3d J_foot_transp_;
  Eigen::Matrix3d J_foot_inv_;
  Eigen::Matrix3d I_;
  Eigen::Vector3d x_err_;
  Eigen::Vector3d x_err_dot_;
  double damp_max_;
  double determinant_max_;


  /** @brief Support temporary Affine3d */
  Eigen::Affine3d tmp_affine3d_;
  /** @brief Support temporary Vector3d */
  Eigen::Vector3d tmp_vector3d_;
  /** @brief Support temporary Matrix3d */
  Eigen::Matrix3d tmp_matrix3d_;
  /** @brief Support temporary Vector6d */
  Eigen::Vector6d tmp_vector6d_;


};

} // namespace

#endif
