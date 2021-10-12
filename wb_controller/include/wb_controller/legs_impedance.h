#ifndef LEGS_IMPEDANCE_H
#define LEGS_IMPEDANCE_H

#include <memory>
#include <Eigen/Core>
#include <wb_controller/gait_generator.h>

namespace wb_controller
{

class LegsImpedance
{

public:

  const std::string CLASS_NAME = "LegsImpedance";

  /**
   * @brief Shared pointer to LegsImpedance
   */
  typedef std::shared_ptr<LegsImpedance> Ptr;

  /**
   * @brief Shared pointer to const LegsImpedance
   */
  typedef std::shared_ptr<const LegsImpedance> ConstPtr;

  LegsImpedance(GaitGenerator::Ptr gait_generator, QuadrupedRobot::Ptr robot_model);

  void update(const Eigen::Matrix3d& Kp_swing_leg, const Eigen::Matrix3d& Kp_stance_leg,
              const Eigen::Matrix3d& Kd_swing_leg, const Eigen::Matrix3d& Kd_stance_leg);

  const Eigen::MatrixXd& getKp() const;
  const Eigen::MatrixXd& getKd() const;

  void startInertiaCompensation(const bool& start);

  std::atomic<bool> inertia_compensation_active_;
  Eigen::MatrixXd M_, Mi_, Kp_postural_, Kd_postural_;

private:


  QuadrupedRobot::Ptr robot_model_;
  GaitGenerator::Ptr gait_generator_;



};


} // namespace

#endif
