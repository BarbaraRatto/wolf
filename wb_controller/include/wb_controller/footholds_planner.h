#ifndef FOOTHOLDS_PLANNER_H
#define FOOTHOLDS_PLANNER_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>
#include <XBotInterface/ModelInterface.h>
#include <wb_controller/locomotion.h>

namespace wb_controller
{

/**
 * @brief This class gets inputs from
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

    FootholdsPlanner(GaitGenerator::Ptr gait_generator, XBot::ModelInterface::Ptr xbot_model, double step_length_max = 0.3, double step_height_max = 0.3);

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
    void setLinearVelocity(const double linear);
    void setAngularVelocity(const double angular);
    void setStepHeight(const double height);
    void setMaxStepHeight(const double max);
    void setMaxStepLength(const double max);
    void increaseStepHeight();
    void decreaseStepHeight();
    void increaseSwingFrequency();
    void decreaseSwingFrequency();

    // Gets
    unsigned int getCmd();
    const Eigen::Matrix3d& getBaseRotationReference() const ;
    const double& getStepLength(const std::string& foot_name);
    const double& getStepHeading(const std::string& foot_name);
    const double& getStepHeight(const std::string& foot_name);
    const double& getStepHeadingRate(const std::string& foot_name);
    const double& getBaseHeight() const ;
    double getLinearVelocity() const ;
    double getAngularVelocity() const ;
    double getStepHeight() const ;
    double getStepLength() const ;

    void setInitialOffsets();

private:

    void calculateFeetStep();

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
    std::atomic<double>  base_linear_velocity_;
    std::atomic<double>  base_angular_velocity_;
    std::atomic<double>  step_height_max_;
    std::atomic<double>  step_length_max_;
    std::atomic<double>  step_height_;

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

    bool offsets_applied_;

    typedef std::map<std::string,double> map_t;
    map_t steps_length_;
    map_t steps_heading_;
    map_t steps_height_;
    map_t steps_heading_rate_;
    Eigen::Matrix3d base_rotation_reference_;
    Eigen::Matrix3d world_R_hf_;

    GaitGenerator::Ptr gait_generator_;
    XBot::ModelInterface::Ptr xbot_model_;

    Eigen::Affine3d world_T_foot_, world_T_base_, base_T_hip_, hip_T_foot_, base_T_foot_;
    std::vector<Eigen::Vector3d> hf_X_initial_footholds_;
    std::vector<Eigen::Vector3d> hf_X_initial_hips_;

    double yaw_base_;
    double step_length_;
};

} // namespace

#endif
