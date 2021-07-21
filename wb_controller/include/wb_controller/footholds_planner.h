#ifndef FOOTHOLDS_PLANNER_H
#define FOOTHOLDS_PLANNER_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>
#include <wb_controller/gait_generator.h>
#include <wb_controller/quadruped_robot.h>

namespace wb_controller
{

class FootholdsPlanner;

class PushRecovery
{

public:
  /**
   * @brief Shared pointer to PushRecovery
   */
  typedef std::shared_ptr<PushRecovery> Ptr;

  PushRecovery(FootholdsPlanner* const footholds_planner_ptr);

  bool update(const double& period);

  const Eigen::Vector2d& getDelta(const std::string& foot_name);

  void setMaxDelta(const double &max);

private:

  FootholdsPlanner* footholds_planner_ptr_;
  bool compute_deltas_;
  double base_mass_;
  double base_length_;
  double base_width_;
  double base_inertia_z_;
  std::map<std::string,Eigen::Vector2d> deltas_;
  std::map<std::string,std::pair<int,int> > signs_;
  Eigen::Vector3d cmd_velocity_;
  Eigen::Vector3d base_velocity_;
  Eigen::Vector3d base_velocity_filt_;
  Eigen::Vector3d error_;
  double max_delta_;
  Eigen::Vector6d base_twist_;
  Eigen::Vector3d com_vel_hf_;
  Eigen::Matrix3d I_hf_;
  Eigen::VectorXd dynamic_th_dot_;
  Eigen::VectorXd static_th_dot_;
  Eigen::Vector3d th_dot_;
  double stop_time_;
  Gait::gait_t prev_gait_;
  double cutoff_freq_;
  // Gains
  double K_pr_lx_;
  double K_pr_ly_;
  double K_pr_r_;

  XBot::Utils::SecondOrderFilter<Eigen::Vector3d> velocity_filter_;

};

/**
 * @brief This class plans the footholds for the feet
 */
class FootholdsPlanner
{

public:

    /**
     * @brief Shared pointer to FootholdsPlanner
     */
    typedef std::shared_ptr<FootholdsPlanner> Ptr;

    /**
     * @brief Shared pointer to const FootholdsPlanner
     */
    typedef std::shared_ptr<const FootholdsPlanner> ConstPtr;

    enum cmd_t {HOLD=0,LINEAR,ANGULAR,LINEAR_AND_ANGULAR,BASE_ONLY,RESET_BASE};

    FootholdsPlanner(GaitGenerator::Ptr gait_generator, QuadrupedRobot::Ptr robot_model, double step_length_max = 0.3, double step_height_max = 0.3);

    void update(const double& period,const Eigen::Vector3d& base_position); // OpenLoop Orientation

    void update(const double& period); // OpenLoop

    void update(const double& period, const Eigen::Vector3d& base_position, const Eigen::Vector3d& base_orientation); // ClosedLoop

    void initializeFeetPosition();

    void initializeFootPosition(const std::string& foot_name);

    // Sets
    void setCmd(const unsigned int cmd);
    void setBasePosition(const Eigen::Vector3d& position);
    void setBaseOrientation(const Eigen::Vector3d& orientation);
    void setDefaultBaseOrientation(const Eigen::Vector3d& orientation);
    void setDefaultBasePosition(const Eigen::Vector3d& position);
    void setBaseVelocityScaleX(const double scale);
    void setBaseVelocityScaleY(const double scale);
    void setBaseVelocityScaleZ(const double scale);
    void setBaseVelocityScaleRoll(const double scale);
    void setBaseVelocityScalePitch(const double scale);
    void setBaseVelocityScaleYaw(const double scale);
    void setLinearVelocityCmd(const double linear);
    void setAngularVelocityCmd(const double angular);
    void setStepHeight(const double height);
    void setMaxStepHeight(const double max);
    void setMaxStepLength(const double max);
    void increaseStepHeight();
    void decreaseStepHeight();
    void increaseSwingFrequency();
    void decreaseSwingFrequency();
    void setComCorrection(const Eigen::Vector2d& delta_com);
    void setComVelocityRef(const Eigen::Vector3d& com_vel_ref);

    // Gets
    unsigned int getCmd();
    const Eigen::Vector3d& getBasePositionReference() const;
    const Eigen::Vector3d& getBaseLinearVelocityReference() const;
    const Eigen::Vector3d& getBaseAngularVelocityReference() const;
    const Eigen::Vector3d& getBaseLinearVelocityReferenceHF() const;
    const Eigen::Vector3d& getBaseAngularVelocityReferenceHF() const;
    const Eigen::Matrix3d& getBaseRotationReference() const;
    const double& getStepLength(const std::string& foot_name);
    const double& getStepHeading(const std::string& foot_name);
    const double& getStepHeight(const std::string& foot_name);
    const double& getStepHeadingRate(const std::string& foot_name);
    const double& getBaseHeight() const;
    double getLinearVelocityCmd() const;
    double getAngularVelocityCmd() const ;
    double getStepHeight() const ;
    double getStepLength() const ;
    const Gait::gait_t& getGaitType() const;
    Eigen::Vector3d& getCurrentFoothold(const std::string& foot_name) ;
    Eigen::Vector3d& getCurrentFootholdHF(const std::string& foot_name) ;
    Eigen::Vector3d& getVirtualFoothold(const std::string& foot_name) ;
    Eigen::Vector3d& getDesiredFoothold(const std::string& foot_name) ;

    void setInitialOffsets();

private:

    void calculateFootSteps();

    void resetFeetStep();

    void resetBaseAngularVelocity();

    void resetBaseLinearVelocity();

    void resetBaseVelocities();

    void resetBasePosition();

    void resetBaseOrientation();

    void resetVelocyScales();

    void calculateBasePosition(const double& period, const Eigen::Vector3d& base_position);

    void calculateBaseOrientation(const double& period, const Eigen::Vector3d& base_orientation);

    // These are modified by the external interface (e.g joypad and dynamic_reconfigure)
    std::atomic<unsigned int> cmd_;
    std::atomic<double>  base_linear_velocity_scale_x_;
    std::atomic<double>  base_linear_velocity_scale_y_;
    std::atomic<double>  base_linear_velocity_scale_z_;
    std::atomic<double>  base_angular_velocity_scale_roll_;
    std::atomic<double>  base_angular_velocity_scale_pitch_;
    std::atomic<double>  base_angular_velocity_scale_yaw_;
    std::atomic<double>  base_linear_velocity_cmd_;
    std::atomic<double>  base_angular_velocity_cmd_;
    std::atomic<double>  step_height_max_;
    std::atomic<double>  step_length_max_;
    std::atomic<double>  step_height_;
    std::atomic<bool>    push_detected_;

    /** @brief Base linear velocity w.r.t horizontal frame
     * (i.e. a frame that has the same position as the base link but oriented as the world except for the yaw which is the same as the base) */
    Eigen::Vector3d hf_base_linear_velocity_;
    /** @brief Base angular velocity */
    Eigen::Vector3d hf_base_angular_velocity_;

    Eigen::Vector3d hf_base_linear_velocity_filt_;
    Eigen::Vector3d hf_base_angular_velocity_filt_;

    Eigen::Vector3d hf_base_linear_velocity_ref_;
    Eigen::Vector3d hf_base_angular_velocity_ref_;

    Eigen::Vector3d base_position_;
    Eigen::Vector3d base_orientation_;
    Eigen::Vector3d default_base_orientation_;
    Eigen::Vector3d default_base_position_;

    Eigen::Vector3d base_position_filt_;
    Eigen::Vector3d base_orientation_filt_;

    Eigen::Vector3d hf_delta_hip_;
    Eigen::Vector3d hf_delta_heding_;
    Eigen::Vector3d hf_delta_foot_;
    Eigen::Vector3d hf_X_current_foothold_;
    Eigen::Vector3d world_delta_hip_;
    Eigen::Vector3d world_delta_foot_;
    Eigen::Vector3d world_X_virtual_foothold_offset_;

    Eigen::Matrix3d world_R_base_;
    Eigen::Matrix3d hf_R_base_;

    Eigen::Vector2d delta_com_;
    Eigen::Vector3d com_vel_ref_;

    bool offsets_applied_;

    typedef std::map<std::string,double> map_t;
    map_t steps_length_;
    map_t steps_heading_;
    map_t steps_height_;
    map_t steps_heading_rate_;
    Eigen::Matrix3d base_rotation_reference_;
    Eigen::Vector3d base_position_reference_;
    Eigen::Vector3d base_linear_velocity_reference_;
    Eigen::Vector3d base_angular_velocity_reference_;
    Eigen::Matrix3d world_R_hf_;

    GaitGenerator::Ptr gait_generator_;
    QuadrupedRobot::Ptr robot_model_;

    Eigen::Affine3d world_T_foot_, world_T_base_, base_T_hip_, hip_T_foot_, base_T_foot_;
    std::vector<Eigen::Vector3d> hf_X_initial_footholds_;
    std::vector<Eigen::Vector3d> hf_X_initial_hips_;

    std::map<std::string,Eigen::Vector3d> desired_foothold_;
    std::map<std::string,Eigen::Vector3d> virtual_foothold_;
    std::map<std::string,Eigen::Vector3d> current_foothold_;
    std::map<std::string,Eigen::Vector3d> current_foothold_hf_;

    double yaw_base_;
    double step_length_;

    friend class PushRecovery;
    PushRecovery::Ptr push_recovery_;
};

} // namespace

#endif
