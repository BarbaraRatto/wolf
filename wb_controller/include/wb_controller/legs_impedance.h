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

  void update();

  // Kp Swing
  void setKpSwingLegHAA(const double& value);
  void setKpSwingLegHFE(const double& value);
  void setKpSwingLegKFE(const double& value);
  // Kd Swing
  void setKdSwingLegHAA(const double& value);
  void setKdSwingLegHFE(const double& value);
  void setKdSwingLegKFE(const double& value);
  // Kp Stance
  void setKpStanceLegHAA(const double& value);
  void setKpStanceLegHFE(const double& value);
  void setKpStanceLegKFE(const double& value);
  // Kd Stance
  void setKdStanceLegHAA(const double& value);
  void setKdStanceLegHFE(const double& value);
  void setKdStanceLegKFE(const double& value);

  const Eigen::MatrixXd& getKp() const;
  const Eigen::MatrixXd& getKd() const;

  void startInertiaCompensation(const bool& start);

  std::atomic<bool> inertia_compensation_active_;
  Eigen::MatrixXd M_, Mi_, Kp_postural_, Kd_postural_;
  Eigen::Matrix3d Kp_swing_leg_, Kd_swing_leg_, Kp_stance_leg_, Kd_stance_leg_;

  void setSwingStanceGains(const Eigen::Vector3d &Kp_swing_leg, const Eigen::Vector3d &Kd_swing_leg, const Eigen::Vector3d &Kp_stance_leg, const Eigen::Vector3d &Kd_stance_leg);

private:

  void loadMatrices();
  void loadValues();

  QuadrupedRobot::Ptr robot_model_;
  GaitGenerator::Ptr gait_generator_;

  std::atomic<double> kp_swing_haa_;
  std::atomic<double> kp_swing_hfe_;
  std::atomic<double> kp_swing_kfe_;
  std::atomic<double> kd_swing_haa_;
  std::atomic<double> kd_swing_hfe_;
  std::atomic<double> kd_swing_kfe_;

  std::atomic<double> kp_stance_haa_;
  std::atomic<double> kp_stance_hfe_;
  std::atomic<double> kp_stance_kfe_;
  std::atomic<double> kd_stance_haa_;
  std::atomic<double> kd_stance_hfe_;
  std::atomic<double> kd_stance_kfe_;

};


} // namespace

#endif
