#ifndef LEGS_KINEMATICS_H
#define LEGS_KINEMATICS_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <wb_controller/geometry.h>
#include <wb_controller/gait_generator.h>
#include <wb_controller/quadruped_robot.h>

namespace wb_controller
{

class LegsKinematics
{

public:

  /**
   * @brief Shared pointer to LegsKinematics
   */
  typedef std::shared_ptr<LegsKinematics> Ptr;

  /**
   * @brief Shared pointer to const LegsKinematics
   */
  typedef std::shared_ptr<const LegsKinematics> ConstPtr;

  LegsKinematics(GaitGenerator::Ptr gait_generator, QuadrupedRobot::Ptr robot_model);

  /**
       * @brief Compute the desired joint positions and velocities
       * @param control period
       * @param current joint positions
       * @return true is the joints position limits are violated
       */
  bool update(const double& period, const Eigen::VectorXd& current_joint_positions);

  void setDesiredBaseHeight(const double& des_base_height);

  bool setJointHomePositions(Eigen::VectorXd& qhome);

  const Eigen::VectorXd& getJointHomePositions();

  /**
       * @brief check if the joints position are between a max and min value, saturate the value if the limits are violated
       * @param q input vector to check
       * @param qmin min values
       * @param qmax max values
       * @return true is the limits are violated
       */
  bool jointLimitsCheck(Eigen::VectorXd& q, const Eigen::VectorXd& qmin,  const Eigen::VectorXd& qmax);

  void setJointLimits(const Eigen::VectorXd& qmax, const Eigen::VectorXd& qmin);

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

  /** @brief True if the control of the base height is active */
  std::atomic<bool> base_height_control_active_;

  std::atomic<double> clik_gain_;

  std::atomic<double> des_base_height_;
  double base_height_;

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
  /** @brief Min joints position */
  Eigen::VectorXd qmin_;
  /** @brief Max joints position */
  Eigen::VectorXd qmax_;
  /** @brief Feet pose w.r.t world */
  Eigen::Affine3d world_T_foot_;

  Eigen::MatrixXd J_;
  Eigen::MatrixXd J_foot_;
  Eigen::Vector3d x_err_;


  /** @brief Support temporary Affine3d */
  Eigen::Affine3d tmp_affine3d_;
  /** @brief Support temporary Vector3d */
  Eigen::Vector3d tmp_vector3d_;
  /** @brief Support temporary Matrix3d */
  Eigen::Matrix3d tmp_matrix3d_;


};

} // namespace

#endif
