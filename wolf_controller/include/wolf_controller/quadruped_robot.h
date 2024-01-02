/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

// ADVR
#include <XBotCoreModel/XBotCoreModel.h>
#include <ModelInterfaceRBDL/ModelInterfaceRBDL.h>
// STD
#include <memory>
#include <atomic>


namespace wolf_controller
{

class QuadrupedRobot : public XBot::ModelInterfaceRBDL
{

public:

  const std::string CLASS_NAME = "QuadrupedRobot";

  typedef std::shared_ptr<QuadrupedRobot> Ptr;

  typedef std::shared_ptr<const QuadrupedRobot> ConstPtr;

  typedef std::map<std::string,std::vector<std::string>>          limb_joint_names_map_t;
  typedef std::map<std::string,std::vector<unsigned int>>         limb_joint_idxs_map_t;
  typedef std::map<std::string,unsigned int>                      joint_idxs_map_t;

  enum robot_states_t {IDLE,INIT,ANOMALY,STANDING_UP,STANDING_DOWN,ACTIVE,N_STATES=7};

  QuadrupedRobot(const std::string& urdf, const std::string& srdf);

  bool update( bool update_position = true,
               bool update_velocity = true,
               bool update_desired_acceleration = true );

  const std::vector<std::string>& getFootNames() const;
  const std::vector<std::string>& getLegNames() const;
  const std::vector<std::string>& getHipNames() const;
  const std::vector<std::string>& getJointNames() const;
  const std::vector<std::string>& getArmNames() const;
  const std::vector<std::string>& getEndEffectorNames() const;
  const std::vector<std::string>& getContactNames() const;
  const std::vector<std::string>& getLimbNames() const;
  const std::string& getBaseLinkName() const;
  const std::string& getImuSensorName() const;
  const std::string& getRobotName() const;
  Eigen::VectorXd getLegJointValues(const Eigen::VectorXd& joints);
  Eigen::VectorXd getArmJointValues(const Eigen::VectorXd& joints);

  const std::vector<unsigned int> &getLimbJointsIds(const std::string& limb_name);

  const unsigned int& getNumberArms() const;
  const unsigned int& getNumberLegs() const;

  const double& getBaseLength() const;
  const double& getBaseWidth() const;

  std::string getStateAsString();
  std::vector<std::string> getStatesAsString();
  robot_states_t getState();
  robot_states_t getPreviousState();
  bool setState(robot_states_t robot_state);

  void getFloatingBasePositionInertia(Eigen::Matrix3d& M);
  void getFloatingBaseOrientationInertia(Eigen::Matrix3d& M);
  void getLimbInertia(const std::string& limb_name, Eigen::MatrixXd& M);
  void getLimbInertiaInverse(const std::string& limb_name, Eigen::MatrixXd& Mi);

  const Eigen::Matrix3d& getBaseRotationInHf() const;
  const Eigen::Matrix3d& getHfRotationInWorld() const;
  const Eigen::Matrix3d& getBaseRotationInWorld() const;
  const Eigen::Vector3d& getBaseRotationInWorldRPY() const;
  const double& getBaseYawInWorld() const;

  using ModelInterface::getPose;
  using ModelInterface::getCOM;
  using ModelInterface::getCOMVelocity;
  using ModelInterface::getJacobian;

  const std::map<std::string, Eigen::Vector3d>& getFeetPositionInWorld() const;
  const std::map<std::string, Eigen::Vector3d>& getFeetPositionInBase() const;
  const std::map<std::string, Eigen::Affine3d>& getFeetPoseInWorld() const;
  const std::map<std::string, Eigen::Affine3d>& getFeetPoseInBase() const;

  Eigen::Vector3d& getFootPositionInWorld(const std::string &name);
  Eigen::Vector3d& getFootPositionInBase(const std::string &name);
  Eigen::Affine3d& getFootPoseInWorld(const std::string &name);
  Eigen::Affine3d& getFootPoseInBase(const std::string &name);

  const std::map<std::string, Eigen::Vector3d>& getEndEffectorsPositionInWorld() const;
  const std::map<std::string, Eigen::Vector3d>& getEndEffectorsPositionInBase() const;
  const std::map<std::string, Eigen::Affine3d>& getEndEffectorsPoseInWorld() const;
  const std::map<std::string, Eigen::Affine3d>& getEndEffectorsPoseInBase() const;

  Eigen::Vector3d& getEndEffectorPositionInWorld(const std::string &name);
  Eigen::Vector3d& getEndEffectorPositionInBase(const std::string &name);
  Eigen::Affine3d& getEndEffectorPoseInWorld(const std::string &name);
  Eigen::Affine3d& getEndEffectorPoseInBase(const std::string &name);

  const Eigen::Affine3d& getBasePoseInWorld() const; // This is the floating base pose w.r.t world

  /**
       * @brief check if the joint velocities are above a max value
       * @param qdot input vector to check
       * @return true is the limits are violated
       */
  std::vector<bool> checkJointVelocities(Eigen::VectorXd &qdot);

  /**
       * @brief check if the joint velocities are above a max value, saturate the value if the limits are violated
       * @param qdot input vector to clamp
       * @return true if the limits are violated
       */
  bool clampJointVelocities(Eigen::VectorXd &qdot);

  /**
       * @brief check if the joint positions are between a max and min value, saturate the value if the limits are violated
       * @param q input vector to clamp
       * @return true if the limits are violated
       */
  bool clampJointPositions(Eigen::VectorXd &q);

  /**
       * @brief check if the joint efforts are above a max value, saturate the value if the limits are violated
       * @param tau input vector to clamp
       * @return true if the limits are violated
       */
  bool clampJointEfforts(Eigen::VectorXd &tau);

  /**
   * @brief get robot's home position when standing up
   * @return q_stand_up
   */
  const Eigen::VectorXd& getStandUpJointPostion();

  /*
   * @brief get robot's home position when standing down
   * @return q_stand_down
   */
  const Eigen::VectorXd& getStandDownJointPostion();

  /**
   * @brief get robot's base height when standing up
   * @return height
   */
  const double& getStandUpHeight();

  /**
   * @brief get robot's base height when standing down
   * @return height
   */
  const double& getStandDownHeight();

  /**
  * @brief Computes the pose of the source_frame w.r.t. the target_frame
  *
  * @param q The joint positions.
  * @param source_frame The source link name.
  * @param target_frame The target link name.
  * @param pose A homogeneous transformation which transforms a point from source frame to target frame
  *       P_target = T * P_source
  * @return True if both source_frame and target_frame are valid. False otherwise.
  */
  bool getPose(const Eigen::VectorXd& q, const std::string& source_frame, const std::string& target_frame, Eigen::Affine3d& pose);

  /**
  * @brief Computes the pose of the source_frame w.r.t. the world frame
  *
  * @param q The joint positions.
  * @param source_frame The source link name.
  * @param pose A homogeneous transformation which transforms a point from source frame to world frame
  *       P_world = T * P_source
  * @return True if source_frame is valid. False otherwise.
  */
  bool getPose(const Eigen::VectorXd& q, const std::string& source_frame, Eigen::Affine3d& pose);

  /**
  * @brief Computes the twist of the source_frame w.r.t. the world frame
  *
  * @param q The joint positions.
  * @param qd The joint velocities.
  * @param source_frame The source link name.
  * @param twist the calculated twist
  * @return True if source_frame is valid. False otherwise.
  */
  bool getTwist(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const std::string& source_frame, Eigen::Vector6d& twist);

  /**
   * @brief Gets the Jacobian of link_name expressed in the world frame, i.e a matrix such that its product with
   * the derivative of the configuration vector gives the velocity twist of link_name (i.e. first linear then angular velocity). The reference point is the origin of the link with link_name name.
   *
   * @param q The joint positions.
   * @param link_name The link name
   * @param J  The Jacobian expressed in the world frame
   * @return True if the link_name and target_frame are valid link names. False otherwise.
   */
  bool getJacobian(const Eigen::VectorXd& q, const std::string& link_name, Eigen::MatrixXd& J);

  /**
   * @brief Gets the Jacobian of link_name expressed in the target_frame, i.e a matrix such that its product with
   * the derivative of the configuration vector gives the velocity twist of link_name according to target_frame
   * (i.e. first linear then angular velocity).
   * The reference point is the origin of the link with link_name name.
   *
   * @param q The joint positions.
   * @param link_name The link name
   * @param target_frame The target frame name
   * @param J  The Jacobian expressed in the world frame
   * @return True if the link_name is a valid link name. False otherwise.
   */
  bool getJacobian(const Eigen::VectorXd& q, const std::string& link_name, const std::string& target_frame, Eigen::MatrixXd& J);

  /**
   * @brief Gets the inertia matrix in joint position q
   *
   * @param q The joint positions.
   * @param M The inertia matrix
   */
  //virtual void getInertiaMatrix(const Eigen::VectorXd& q, Eigen::MatrixXd& M) const;


  /**
   * @brief Gets the current robot's height
   *
   * @return robot's height
   */
  double getCurrentHeight();


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
  std::string imu_name_;
  std::string robot_name_;

  limb_joint_idxs_map_t joint_limb_idx_;
  std::vector<unsigned int> joint_leg_idx_;
  std::vector<unsigned int> joint_arm_idx_;

  joint_idxs_map_t joint_idx_;

  limb_joint_names_map_t joint_leg_names_;
  limb_joint_names_map_t joint_arm_names_;

  Eigen::VectorXd joint_legs_;
  Eigen::VectorXd joint_arms_;

  unsigned int n_legs_;
  unsigned int n_arms_;

  double base_length_;
  double base_width_;

  Eigen::Affine3d world_T_base_;
  Eigen::Matrix3d world_R_hf_;
  Eigen::Matrix3d world_R_base_;
  Eigen::Matrix3d hf_R_base_;
  Eigen::Vector3d world_RPY_base_;
  double base_yaw_;

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
  Eigen::VectorXd q_stand_up_;
  /** @brief Homing position when standing down */
  Eigen::VectorXd q_stand_down_;
  /** @brief Base's height when standing up */
  double stand_up_height_;
  /** @brief Base's height when standing down */
  double stand_down_height_;

  std::atomic<robot_states_t> robot_state_;
  std::atomic<robot_states_t> robot_state_prev_;
  std::string robot_state_string_;

  mutable RigidBodyDynamics::Model virtual_model_;

  Eigen::MatrixXd tmp_Mi_;
  Eigen::MatrixXd tmp_M_;
  Eigen::MatrixXd tmp_jacobian_;
  Eigen::Matrix3d tmp_matrix3d_;
  Eigen::Vector3d tmp_vector3d_;
  Eigen::Affine3d tmp_affine3d_;
  Eigen::Affine3d tmp_affine3d_1_;
  mutable KDL::Jacobian tmp_kdl_jacobian_;
  mutable KDL::Rotation tmp_kdl_rotation_;

};

} //@namespace wolf_controller

#endif //QUADRUPED_ROBOT_H
