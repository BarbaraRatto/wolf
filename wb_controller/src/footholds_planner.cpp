#include <wb_controller/footholds_planner.h>

using namespace rt_logger;

namespace wb_controller {

#define CLASS_NAME "FootholdsPlanner"

FootholdsPlanner::FootholdsPlanner(GaitGenerator::Ptr gait_generator, XBot::ModelInterface::Ptr xbot_model, double step_length_max, double step_height_max)
{

    assert(gait_generator);
    gait_generator_ = gait_generator;
    assert(xbot_model);
    xbot_model_ = xbot_model;
    assert(step_length_max>=0.0);
    step_length_max_ = step_length_max;
    assert(step_height_max>=0.0);
    step_height_max_ = step_height_max;

    const std::vector<std::string>& feet_names = gait_generator_->getFeetNames();
    for(unsigned int i=0;i<feet_names.size();i++)
    {
        steps_length_[feet_names[i]] = 0.0;
        steps_heading_[feet_names[i]] = 0.0;
        steps_height_[feet_names[i]] = 0.0;
    }

    resetVelocyScales();

    base_linear_velocity_ = 0.0; // [m/s]
    base_angular_velocity_ = 0.0; // [rad/s]

    base_rotation_reference_ = Eigen::Matrix3d::Identity();
    base_position_ = base_orientation_ = Eigen::Vector3d::Zero();

    cmd_ = cmd_t::HOLD;

    hf_X_initial_footholds_.resize(4);
    hf_X_initial_hips_.resize(4); // \f$X_hip(i)\f$ with i corresponding to the leg number
    for(unsigned int i=0; i<4; i++)
    {
        hf_X_initial_hips_[i].setZero();
        hf_X_initial_footholds_[i].setZero();
    }

    step_length_ = 0.0;
    step_height_ = 0.0;

    offsets_applied_ = false;

    RtLogger::getLogger().addPublisher(CLASS_NAME"/desired_height",base_position_(2));
}

void FootholdsPlanner::update(const double& period,const Eigen::Vector3d& base_position) // OpenLoop Orientation
{
    update(period,base_position,base_orientation_);
}

void FootholdsPlanner::update(const double& period) // OpenLoop
{
    update(period,base_position_,base_orientation_);
}

void FootholdsPlanner::initializeFootPosition(const std::string& foot_name)
{
    xbot_model_->getPose(foot_name,world_T_foot_);
    gait_generator_->setInitialPose(foot_name,world_T_foot_);
}

void FootholdsPlanner::initializeFeetPosition()
{
    const std::vector<std::string>& feet_names = gait_generator_->getFeetNames();

    for(unsigned int i=0; i<feet_names.size(); i++)
        initializeFootPosition(feet_names[i]);
}

void FootholdsPlanner::update(const double& period, const Eigen::Vector3d& base_position, const Eigen::Vector3d& base_orientation) // ClosedLoop
{
    unsigned int cmd = cmd_;

    ROS_DEBUG_NAMED(CLASS_NAME,"update");

    xbot_model_->getPose("base_link",world_T_base_);

    ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_T_base_.translation()" << world_T_base_.translation());
    ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_T_base_.linear()" << world_T_base_.linear());

    world_R_hf_ = Eigen::Matrix3d::Identity();
    yaw_base_ = std::atan2(world_T_base_.linear()(1,0),world_T_base_.linear()(0,0));
    world_R_hf_ = Eigen::AngleAxisd(yaw_base_,Eigen::Vector3d::UnitZ());

    ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"yaw_base_" << yaw_base_);
    ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_R_hf_" << world_R_hf_);

    world_R_base_ = world_T_base_.linear();
    hf_R_base_ = world_R_hf_.transpose() * world_R_base_;

    ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_R_base_" << world_R_base_);
    ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_R_base_" << hf_R_base_);

    setInitialOffsets();

    switch(cmd)
    {

    case cmd_t::HOLD:
        resetBaseVelocities();
        resetVelocyScales();
        calculateBasePosition(period,base_position);
        calculateBaseOrientation(period,base_orientation);
        resetFeetStep();
        break;

    case cmd_t::LINEAR:
        calculateBasePosition(period,base_position);
        resetBaseAngularVelocity();
        calculateFeetStep();
        break;

    case cmd_t::ANGULAR:
        calculateBaseOrientation(period,base_orientation);
        resetBaseLinearVelocity();
        calculateFeetStep();
        break;

    case cmd_t::LINEAR_AND_ANGULAR:
        calculateBasePosition(period,base_position);
        calculateBaseOrientation(period,base_orientation);
        calculateFeetStep();
        break;

    case cmd_t::BASE_ONLY:
        calculateBasePosition(period,base_position);
        calculateBaseOrientation(period,base_orientation);
        resetFeetStep();
        break;

    case cmd_t::RESET_BASE:
        resetBaseVelocities();
        resetBasePosition();
        resetBaseOrientation();
        resetFeetStep();
        break;
    };

    const std::vector<std::string>& feet_names = gait_generator_->getFeetNames();
    for(unsigned int i=0; i<feet_names.size(); i++)
    {
        // Set the initial pose for the next swing
        if(gait_generator_->isLiftOff(feet_names[i]))
            initializeFootPosition(feet_names[i]);

        gait_generator_->setStepLength(feet_names[i], steps_length_[feet_names[i]]);
        gait_generator_->setStepHeading(feet_names[i], steps_heading_[feet_names[i]]);
        gait_generator_->setStepHeight(feet_names[i], steps_height_[feet_names[i]]);
        gait_generator_->setStepHeadingRate(feet_names[i], steps_heading_rate_[feet_names[i]]);
    }

    // Update the gait_generator
    gait_generator_->update(period);
}

void FootholdsPlanner::calculateFeetStep()
{
    const std::vector<std::string>& feet_names = gait_generator_->getFeetNames();

    for(unsigned int i=0; i<feet_names.size(); i++)
    {
        //if(gait_generator_->isLiftOff(feet_names[i]))
        if(gait_generator_->isSwinging(feet_names[i]))
        {
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"*********");
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"CalculateFeetStep for foot "<<feet_names[i]);

            // 1) Compute the displacement of the foot produced by the linear velocity command
            hf_delta_hip_.setZero(); // \f$\deltaL_{x,y,0}\f$
            hf_delta_hip_(0) = hf_base_linear_velocity_(0)*1.0/gait_generator_->getSwingFrequency(feet_names[i]);
            hf_delta_hip_(1) = hf_base_linear_velocity_(1)*1.0/gait_generator_->getSwingFrequency(feet_names[i]);
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_delta_hip_ (Linear part): "<<hf_delta_hip_.transpose());

            // 2) Compute the displacement of the foot produced by the angular velocity command
            hf_delta_heding_.setZero(); // \f$\deltaL_{h,0}\f$
            hf_delta_heding_(2) = hf_base_angular_velocity_(2)*1.0/gait_generator_->getSwingFrequency(feet_names[i]);
            hf_delta_heding_ = hf_delta_heding_.cross(hf_X_initial_hips_[i]);
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_delta_heding_ (Angular part): "<<hf_delta_heding_.transpose());

            // 3) Combine the two displacements
            hf_delta_hip_(0)+= hf_delta_heding_(0);
            hf_delta_hip_(1)+= hf_delta_heding_(1);
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_delta_hip_ (Combined): "<<hf_delta_hip_.transpose());

            // 4) Calculate the foothold offset based on the initial feet position (virtual foothold offset)
            xbot_model_->getPose(feet_names[i],"base_link",base_T_foot_);
            // current foot position in the horizontal frame
            hf_X_current_foothold_ = hf_R_base_ * base_T_foot_.translation();
            //world_X_virtual_foothold_offset_ = world_R_hf_ * (hf_X_initial_footholds_[i] - hf_X_current_foothold_);
            //world_X_virtual_foothold_offset_(2) = 0;
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_X_current_foothold_: "<<hf_X_current_foothold_.transpose());

            // 5) Sum everything to obtain the new foothold displacement w.r.t hf
            hf_delta_foot_.setZero();
            hf_delta_foot_.head(2) =  hf_delta_hip_.head(2)  + (hf_X_initial_footholds_[i] - hf_X_current_foothold_).head(2);
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_delta_foot_: "<<hf_delta_foot_.transpose());

            // 6) Sum everything to obtain the new foothold displacement w.r.t world
            //world_delta_foot_.setZero();
            //world_delta_foot_.head(2) =  world_R_hf_ * hf_delta_foot_;
            //ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_delta_foot_: "<<world_delta_foot_.transpose());

            // 7) Get the step length and heading
            step_length_ = std::sqrt(hf_delta_foot_(0)*hf_delta_foot_(0) + hf_delta_foot_(1)*hf_delta_foot_(1));

            if(step_length_ > step_length_max_)
            {
                step_length_ = step_length_max_;
                ROS_WARN_STREAM_NAMED(CLASS_NAME,"Step length is greater than: "<<step_length_max_);
            }

            steps_length_[feet_names[i]]         = step_length_;
            steps_heading_[feet_names[i]]        = std::atan2(hf_delta_foot_(1),hf_delta_foot_(0));
            steps_height_[feet_names[i]]         = step_height_;
            steps_heading_rate_[feet_names[i]]   = hf_base_angular_velocity_(2);

            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"steps_length["<<feet_names[i]<<"]: "<<steps_length_[feet_names[i]]);
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"steps_heading_["<<feet_names[i]<<"]: "<<steps_heading_[feet_names[i]]);
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"steps_height_["<<feet_names[i]<<"]: "<<steps_height_[feet_names[i]]);
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"steps_heading_rate_["<<feet_names[i]<<"]: "<<steps_heading_rate_[feet_names[i]]);
        }
        else // if(gait_generator_->isTouchDown(feet_names[i]))
        {
            steps_length_[feet_names[i]]         = 0.0;
            steps_heading_[feet_names[i]]        = 0.0;
            steps_height_[feet_names[i]]         = 0.0;
            steps_heading_rate_[feet_names[i]]   = 0.0;
        }
    }

    gait_generator_->activateSwing();
}

void FootholdsPlanner::resetFeetStep()
{
    const std::vector<std::string>& feet_names = gait_generator_->getFeetNames();

    for(unsigned int i=0; i<feet_names.size(); i++)
    {
        steps_length_[feet_names[i]]   = 0.0;
        steps_heading_[feet_names[i]] = 0.0;
        steps_height_[feet_names[i]]   = 0.0;
        steps_heading_rate_[feet_names[i]]   = 0.0;
    }
    gait_generator_->deactivateSwing();
}

void FootholdsPlanner::resetBaseAngularVelocity()
{
    hf_base_angular_velocity_.setZero();
    hf_base_angular_velocity_ref_.setZero();
    hf_base_angular_velocity_filt_.setZero();
}

void FootholdsPlanner::resetBaseLinearVelocity()
{
    hf_base_linear_velocity_.setZero();
    hf_base_linear_velocity_ref_.setZero();
    hf_base_linear_velocity_filt_.setZero();
}

void FootholdsPlanner::resetBaseVelocities()
{
    resetBaseAngularVelocity();
    resetBaseLinearVelocity();
}

void FootholdsPlanner::resetBasePosition()
{
    for(unsigned int i=0;i<3;i++)
        base_position_(i) = secondOrderFilter(base_position_(i),base_position_filt_(i),default_base_position_(i),1.0);
}

void FootholdsPlanner::resetBaseOrientation()
{
    default_base_orientation_(2) = base_orientation_(2); // Keep the same yaw

    for(unsigned int i=0;i<3;i++)
        base_orientation_(i) = secondOrderFilter(base_orientation_(i),base_orientation_filt_(i),default_base_orientation_(i),1.0); //FIXME hardcoded gain, it should be based on the sampling time

    rpyToRot(base_orientation_,base_rotation_reference_);
    base_rotation_reference_.transposeInPlace();
}

void FootholdsPlanner::resetVelocyScales()
{
    base_linear_velocity_scale_x_ = 0.0;
    base_linear_velocity_scale_y_ = 0.0;
    base_linear_velocity_scale_z_ = 0.0;

    base_angular_velocity_scale_roll_ = 0.0;
    base_angular_velocity_scale_pitch_ = 0.0;
    base_angular_velocity_scale_yaw_ = 0.0;
}

void FootholdsPlanner::calculateBasePosition(const double& period, const Eigen::Vector3d& base_position)
{
    base_position_ = base_position;

    hf_base_linear_velocity_ref_(0) = base_linear_velocity_ * base_linear_velocity_scale_x_;
    hf_base_linear_velocity_ref_(1) = base_linear_velocity_ * base_linear_velocity_scale_y_;
    hf_base_linear_velocity_ref_(2) = base_linear_velocity_ * base_linear_velocity_scale_z_;

    for(unsigned int i=0;i<3;i++)
        hf_base_linear_velocity_(i) = secondOrderFilter(hf_base_linear_velocity_(i),hf_base_linear_velocity_filt_(i),hf_base_linear_velocity_ref_(i),0.5); //FIXME hardcoded gain, it should be based on the sampling time

    base_position_ = world_R_hf_ * hf_base_linear_velocity_ * period + base_position_;

    // This is the base height computed w.r.t world
    //base_position_(2);
}

void FootholdsPlanner::calculateBaseOrientation(const double& period, const Eigen::Vector3d& base_orientation)
{
    base_orientation_ = base_orientation;

    hf_base_angular_velocity_ref_(0) = base_angular_velocity_ * base_angular_velocity_scale_roll_;
    hf_base_angular_velocity_ref_(1) = base_angular_velocity_ * base_angular_velocity_scale_pitch_;
    hf_base_angular_velocity_ref_(2) = base_angular_velocity_ * base_angular_velocity_scale_yaw_;

    for(unsigned int i=0;i<3;i++)
        hf_base_angular_velocity_(i) = secondOrderFilter(hf_base_angular_velocity_(i),hf_base_angular_velocity_filt_(i),hf_base_angular_velocity_ref_(i),0.5);

    base_orientation_ = hf_base_angular_velocity_ * period + base_orientation_;

    // This is the base rotation computed w.r.t world
    rpyToRot(base_orientation_,base_rotation_reference_);
    base_rotation_reference_.transposeInPlace();
}

void FootholdsPlanner::setInitialOffsets()
{
    if(!offsets_applied_)
    {
        const std::vector<std::string>& hips_names = gait_generator_->getHipsNames();
        for(unsigned int i=0; i<hips_names.size(); i++)
        {
            xbot_model_->getPose(gait_generator_->getFeetNames()[i],"base_link",base_T_foot_);
            // initial feet offsets in the horizontal frame
            hf_X_initial_footholds_[i] = hf_R_base_ * base_T_foot_.translation();
            // initial hip positions, we assume the base starts horizontal (TODO)
            hf_X_initial_hips_[i] = base_T_hip_.translation();
        }

        offsets_applied_ = true;
    }
}

// Sets
void FootholdsPlanner::setCmd(const unsigned int cmd)
{
    cmd_ = cmd;
}

void FootholdsPlanner::setBasePosition(const Eigen::Vector3d& position)
{
    base_position_ = position;
}

void FootholdsPlanner::setBaseOrientation(const Eigen::Vector3d& orientation)
{
    base_orientation_ = orientation;
}

void FootholdsPlanner::setDefaultBaseOrientation(const Eigen::Vector3d& orientation)
{
    default_base_orientation_ = orientation;
}

void FootholdsPlanner::setDefaultBasePosition(const Eigen::Vector3d& position)
{
    default_base_position_ = position;
}

void FootholdsPlanner::setBaseVelocityScaleX(const double scale)
{
    base_linear_velocity_scale_x_ = scale;
}

void FootholdsPlanner::setBaseVelocityScaleY(const double scale)
{
    base_linear_velocity_scale_y_ = scale;
}

void FootholdsPlanner::setBaseVelocityScaleZ(const double scale)
{
    base_linear_velocity_scale_z_ = scale;
}

void FootholdsPlanner::setBaseVelocityScaleRoll(const double scale)
{
    base_angular_velocity_scale_roll_ = scale;
}

void FootholdsPlanner::setBaseVelocityScalePitch(const double scale)
{
    base_angular_velocity_scale_pitch_ = scale;
}

void FootholdsPlanner::setBaseVelocityScaleYaw(const double scale)
{
    base_angular_velocity_scale_yaw_ = scale;
}

void FootholdsPlanner::increaseStepHeight()
{
    setStepHeight(step_height_ + 0.01); // Increase step height
}

void FootholdsPlanner::decreaseStepHeight()
{
  setStepHeight(step_height_ - 0.01); // Decrease step height
}

void FootholdsPlanner::setLinearVelocity(const double linear)
{
    base_linear_velocity_ = linear;
}

void FootholdsPlanner::setAngularVelocity(const double angular)
{
    base_angular_velocity_ = angular;
}

void FootholdsPlanner::setStepHeight(const double height)
{
    if(height > step_height_max_) // Check if it is ok
    {
        double height_max = step_height_max_;
        step_height_ = height_max;
        ROS_WARN_STREAM_NAMED(CLASS_NAME,"Step height is greater than: "<<height_max);
    }
    else if(height <= 0.0)
    {
        step_height_ = 0.0;
        ROS_WARN_NAMED(CLASS_NAME,"Step height is less equal than: 0.0");
    }
    else
    {
        step_height_ = height;
        ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set step height to: "<<height);
    }
}

void FootholdsPlanner::setMaxStepHeight(const double max)
{
    if(max >= 0.0) // Check if it is ok
    {
        step_height_max_ = max;
    }
    else
        ROS_WARN_NAMED(CLASS_NAME,"Max step height is less equal than: 0.0");
}

void FootholdsPlanner::setMaxStepLength(const double max)
{
    if(max >= 0.0) // Check if it is ok
    {
        step_length_max_ = max;
    }
    else
        ROS_WARN_NAMED(CLASS_NAME,"Max step length is less equal than: 0.0");
}

// Gets
unsigned int FootholdsPlanner::getCmd()
{
    return cmd_;
}

const Eigen::Matrix3d& FootholdsPlanner::getBaseRotationReference() const
{
    return base_rotation_reference_;
}

const double& FootholdsPlanner::getStepLength(const std::string& foot_name)
{
    return steps_length_[foot_name];
}

const double& FootholdsPlanner::getStepHeading(const std::string& foot_name)
{
    return steps_heading_[foot_name];
}

const double& FootholdsPlanner::getStepHeight(const std::string& foot_name)
{
    return steps_height_[foot_name];
}

const double& FootholdsPlanner::getStepHeadingRate(const std::string& foot_name)
{
    return steps_heading_rate_[foot_name];
}

const double& FootholdsPlanner::getBaseHeight() const
{
    return base_position_(2);
}

double FootholdsPlanner::getLinearVelocity() const
{
    return base_linear_velocity_;
}

double FootholdsPlanner::getAngularVelocity() const
{
    return base_angular_velocity_;
}

double FootholdsPlanner::getStepHeight() const
{
    return step_height_;
}

double FootholdsPlanner::getStepLength() const
{
    return step_length_;
}

}; // namespace
