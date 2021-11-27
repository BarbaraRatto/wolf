#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

// ADVR
#include <XBotCoreModel/XBotCoreModel.h>
#include <ModelInterfaceRBDL/ModelInterfaceRBDL.h>
// STD
#include <memory>
#include <atomic>


namespace wb_controller
{

class QuadrupedRobot : public XBot::ModelInterfaceRBDL
{

public:

  const std::string CLASS_NAME = "QuadrupedRobot";

  typedef std::shared_ptr<QuadrupedRobot> Ptr;

  typedef std::shared_ptr<const QuadrupedRobot> ConstPtr;

  typedef std::map<std::string,std::vector<std::string> > limb_joint_names_map_t;
  typedef std::map<std::string,std::vector<int> >         limb_joint_idxs_map_t;
  typedef std::map<std::string,int >                      joint_idxs_map_t;

  enum robot_states_t {INIT,WALKING,MANIPULATION};

  QuadrupedRobot(const std::string& urdf, const std::string& srdf);

  bool update( bool update_position = true,
               bool update_velocity = true,
               bool update_desired_acceleration = true );

  const std::vector<std::string>& getFootNames() const;
  const std::vector<std::string>& getLegNames() const;
  const std::vector<std::string>& getHipNames() const;
  const std::vector<std::string>& getJointNames() const;
  const std::vector<std::string>& getEndEffectorNames() const;
  const std::vector<std::string>& getContactNames() const;
  const std::vector<std::string>& getLimbNames() const;
  const std::string& getBaseLinkName() const;

  const std::vector<int>& getLimbJointsIds(const std::string& limb_name);

  const unsigned int& getNumberArms() const;
  const unsigned int& getNumberLegs() const;

  const double& getBaseLength() const;
  const double& getBaseWidth() const;

  robot_states_t getState();
  bool setState(robot_states_t robot_state);

  void getFloatingBasePositionInertia(Eigen::Matrix3d& M);
  void getFloatingBaseOrientationInertia(Eigen::Matrix3d& M);
  void getLimbInertia(const std::string& limb_name, Eigen::MatrixXd& M);
  void getLimbInertiaInverse(const std::string& limb_name, Eigen::MatrixXd& Mi);

  const Eigen::Matrix3d& getBaseRotationInHf() const;
  const Eigen::Matrix3d& getHfRotationInWorld() const;
  const Eigen::Matrix3d& getBaseRotationInWorld() const;
  const double& getHfYawInWorld() const;

  using ModelInterface::getPose;
  using ModelInterface::getCOM;
  using ModelInterface::getCOMVelocity;
  using ModelInterface::getJacobian;

  const std::map<std::string, Eigen::Vector3d> &getFeetPositionInWorld() const;
  const std::map<std::string, Eigen::Vector3d> &getFeetPositionInBase() const;
  const std::map<std::string, Eigen::Affine3d> &getFeetPoseInWorld() const;
  const std::map<std::string, Eigen::Affine3d> &getFeetPoseInBase() const;

  Eigen::Vector3d &getFootPositionInWorld(const std::string &name);
  Eigen::Vector3d &getFootPositionInBase(const std::string &name);
  Eigen::Affine3d &getFootPoseInWorld(const std::string &name);
  Eigen::Affine3d &getFootPoseInBase(const std::string &name);

  const std::map<std::string, Eigen::Vector3d> &getEndEffectorsPositionInWorld() const;
  const std::map<std::string, Eigen::Vector3d> &getEndEffectorsPositionInBase() const;
  const std::map<std::string, Eigen::Affine3d> &getEndEffectorsPoseInWorld() const;
  const std::map<std::string, Eigen::Affine3d> &getEndEffectorsPoseInBase() const;

  Eigen::Vector3d &getEndEffectorPositionInWorld(const std::string &name);
  Eigen::Vector3d &getEndEffectorPositionInBase(const std::string &name);
  Eigen::Affine3d &getEndEffectorPoseInWorld(const std::string &name);
  Eigen::Affine3d &getEndEffectorPoseInBase(const std::string &name);

  const Eigen::Affine3d &getBasePoseInWorld() const; // This is the floating base pose w.r.t world

  /**
       * @brief check if the joint velocities are above a max value, saturate the value if the limits are violated
       * @param qdot input vector to check
       * @return true is the limits are violated
       */
  bool clampJointVelocities(Eigen::VectorXd &qdot);

  /**
       * @brief check if the joint positions are between a max and min value, saturate the value if the limits are violated
       * @param q input vector to check
       * @return true is the limits are violated
       */
  bool clampJointPositions(Eigen::VectorXd &q);

  /**
       * @brief check if the joint efforts are above a max value, saturate the value if the limits are violated
       * @param tau input vector to check
       * @return true is the limits are violated
       */
  bool clampJointEfforts(Eigen::VectorXd &tau);

  /**
       * @brief get robot's home position when standing up
       * @return qhome
       */
  const Eigen::VectorXd& getJointHomePositions();

private:

  std::vector<std::string> foot_names_; // foot tip names
  std::vector<std::string> hip_names_;
  std::vector<std::string> leg_names_;
  std::vector<std::string> arm_names_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> ee_names_; // end-effector names
  std::vector<std::string> contact_names_; // foot + arm names
  std::vector<std::string> limb_names_; // chain names
  std::string base_name_;

  limb_joint_idxs_map_t joint_limb_idx_;

  joint_idxs_map_t joint_idx_;

  limb_joint_names_map_t joint_legs_;
  limb_joint_names_map_t joint_arms_;

  unsigned int n_legs_;
  unsigned int n_arms_;

  double base_length_;
  double base_width_;

  Eigen::Affine3d world_T_base_;
  Eigen::Matrix3d world_R_hf_;
  Eigen::Matrix3d world_R_base_;
  Eigen::Matrix3d hf_R_base_;
  double yaw_base_;

  /** @brief Foot positions w.r.t base */
  std::map<std::string,Eigen::Vector3d> base_X_foot_;
  /** @brief Foot positions w.r.t world */
  std::map<std::string,Eigen::Vector3d> world_X_foot_;
  /** @brief Foot pose w.r.t base */
  std::map<std::string,Eigen::Affine3d> base_T_foot_;
  /** @brief Foot pose w.r.t world */
  std::map<std::string,Eigen::Affine3d> world_T_foot_;
  /** @brief Arm end-effector positions w.r.t base */
  std::map<std::string,Eigen::Vector3d> base_X_ee_;
  /** @brief Arm end-effector positions w.r.t world */
  std::map<std::string,Eigen::Vector3d> world_X_ee_;
  /** @brief Arm end-effector pose w.r.t base */
  std::map<std::string,Eigen::Affine3d> base_T_ee_;
  /** @brief Arm end-effector pose w.r.t world */
  std::map<std::string,Eigen::Affine3d> world_T_ee_;
  /** @brief Min joints position */
  Eigen::VectorXd q_min_;
  /** @brief Max joints position */
  Eigen::VectorXd q_max_;
  /** @brief Max joints velocity */
  Eigen::VectorXd qdot_max_;
  /** @brief Max joints effort */
  Eigen::VectorXd tau_max_;
  /** @brief Homing position when standing up */
  Eigen::VectorXd qhome_;

  std::atomic<robot_states_t> robot_state_;

  Eigen::MatrixXd tmp_Mi_;
  Eigen::MatrixXd tmp_M_;

};

} //@namespace wb_controller

#endif //QUADRUPED_ROBOT_H
