#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

// ADVR
#include <XBotCoreModel/XBotCoreModel.h>
#include <XBotInterface/ModelInterface.h>
// STD
#include <memory>

namespace wb_controller
{

class QuadrupedRobot : public XBot::ModelInterface
{

public:

  const std::string CLASS_NAME = "QuadrupedRobot";

  typedef std::shared_ptr<QuadrupedRobot> Ptr;

  typedef std::shared_ptr<const QuadrupedRobot> ConstPtr;

  enum robot_states_t {INIT,WALKING,MANIPULATION};

  QuadrupedRobot(const std::string& urdf, const std::string& srdf);

  const std::vector<std::string>& getFootNames() const;
  const std::vector<std::string>& getHipNames() const;
  const std::vector<std::string>& getJointNames() const;
  const std::vector<std::string>& getArmNames() const;
  const std::vector<std::string>& getContactNames() const;
  const std::vector<std::string>& getLimbNames() const;
  XBot::ModelInterface::Ptr getModelImp();

  const unsigned int& getNumberArms() const;
  const unsigned int& getNumberLegs() const;

  const double& getBaseLength() const;
  const double& getBaseWidth() const;

  const std::string& getBaseLinkName() const;

  robot_states_t getRobotState();
  bool setRobotState(robot_states_t robot_state);

  const Eigen::Matrix3d& getFloatingBaseInertia();

  virtual void getModelOrderedJoints( std::vector<std::string>& joint_name ) const ;

  virtual bool setFloatingBasePose( const KDL::Frame& floating_base_pose ) ;

  virtual bool setFloatingBaseTwist( const KDL::Twist& floating_base_twist ) ;

  virtual bool getFloatingBaseLink( std::string& floating_base_link) const;

  virtual bool getPose( const std::string& source_frame,
                        KDL::Frame& pose ) const;
  virtual bool getJacobian( const std::string& link_name,
                            const KDL::Vector& reference_point,
                            KDL::Jacobian& J) const;
  virtual bool getVelocityTwist( const std::string& link_name,
                                 KDL::Twist& velocity) const;

  virtual bool getAccelerationTwist( const std::string& link_name,
                                     KDL::Twist& acceleration) const  ;

  virtual bool getRelativeAccelerationTwist(const std::string& link_name, const std::string& base_link_name,
                                            KDL::Twist& acceleration) const  ;

  virtual bool getPointAcceleration(const std::string& link_name,
                                    const KDL::Vector& point,
                                    KDL::Vector& acceleration) const  ;

  virtual bool computeRelativeJdotQdot(const std::string& target_link_name,
                                       const std::string& base_link_name,
                                       KDL::Twist& jdotqdot) const  ;

  virtual void getCOM( KDL::Vector& com_position ) const  ;

  virtual void getCOMVelocity( KDL::Vector& velocity) const  ;

  virtual void getCOMAcceleration( KDL::Vector& acceleration) const  ;

  virtual void getCentroidalMomentum(Eigen::Vector6d& centroidal_momentum) const  ;

  virtual double getMass() const  ;

  virtual void getGravity( KDL::Vector& gravity ) const  ;


  virtual void setGravity( const KDL::Vector& gravity )  ;


  virtual void getInertiaMatrix(Eigen::MatrixXd& M) const  ;

  virtual void computeGravityCompensation( Eigen::VectorXd& g ) const  ;

  virtual void computeNonlinearTerm( Eigen::VectorXd& n ) const  ;

  virtual void computeInverseDynamics( Eigen::VectorXd& tau) const  ;

  virtual int getLinkID(const std::string& link_name) const  ;

  virtual bool init_model(const XBot::ConfigOptions& config)  ;

  virtual bool update_internal( bool update_position,
                                bool update_velocity,
                                bool update_desired_acceleration)  ;

private:

  XBot::ModelInterface::Ptr model_imp_;
  std::vector<std::string> foot_names_; // foot tip names
  std::vector<std::string> hip_names_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> arm_names_; // arm tip names
  std::vector<std::string> contact_names_; // foot + arm names
  std::vector<std::string> limb_names_; // chain names
  std::vector<std::string> base_names_;

  unsigned int n_legs_;
  unsigned int n_arms_;

  double base_length_;
  double base_width_;

  Eigen::MatrixXd M_;
  Eigen::Matrix3d Ifb_;

  std::atomic<robot_states_t> robot_state_;

};

} //@namespace wb_controller

#endif //QUADRUPED_ROBOT_H
