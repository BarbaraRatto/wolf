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

  bool update(const Eigen::VectorXd &current_joint_positions);

  const Eigen::VectorXd& getDesiredJointPositions();

  const Eigen::VectorXd& getDesiredJointVelocities();

  void reset();

private:

  QuadrupedRobot::Ptr robot_model_;

  GaitGenerator::Ptr gait_generator_;

  TerrainEstimator::Ptr terrain_estimator_;

  Eigen::MatrixXd J_;
  Eigen::Matrix3d J_foot_;
  Eigen::Matrix3d J_foot_inv_;

  /** @brief Desired joint positions */
  Eigen::VectorXd des_joint_positions_;
  /** @brief Desired joint velocities */
  Eigen::VectorXd des_joint_velocities_;
  /** @brief Homing position */
  Eigen::VectorXd qhome_;
  /** @brief Joint position */
  Eigen::VectorXd q_;
  /** @brief base in world */
  Eigen::Affine3d world_T_base_;
  /** @brief foot in world */
  Eigen::Affine3d world_T_foot_;
  /** @brief world adjustment */
  Eigen::Vector3d world_adj_;
  /** @brief base adjustment */
  Eigen::Vector3d base_adj_;

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
