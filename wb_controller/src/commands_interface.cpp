#include <wb_controller/commands_interface.h>

namespace wb_controller {

#define CLASS_NAME "CommandsInterface"

CommandsInterface::CommandsInterface(GaitGenerator::Ptr gait_generator, XBot::ModelInterface::Ptr xbot_model, double step_length_max, double step_height_max)
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

    base_linear_velocity_scale_x_ = 0.0;
    base_linear_velocity_scale_y_ = 0.0;
    base_linear_velocity_scale_z_ = 0.0;

    base_angular_velocity_scale_roll_ = 0.0;
    base_angular_velocity_scale_pitch_ = 0.0;
    base_angular_velocity_scale_yaw_ = 0.0;

    base_linear_velocity_max_ = 0.0; // [m/s]
    base_angular_velocity_max_ = 0.0; // [rad/s]

    base_rotation_reference_ = Eigen::Matrix3d::Identity();
    base_position_ = base_orientation_ = Eigen::Vector3d::Zero();

    cmd_ = cmd_t::HOLD;

    hf_X_hip_foot_offsets_.resize(4);
    hf_X_virtual_hips_.resize(4);
    for(unsigned int i=0; i<4; i++)
    {
        hf_X_hip_foot_offsets_[i].setZero();
        hf_X_virtual_hips_[i].setZero();
    }

    step_length_ = 0.0;
    step_height_ = 0.0;

    offset_applied_ = false;
}

void CommandsInterface::update(const double& period,const Eigen::Vector3d& base_position) // OpenLoop Orientation
{
    update(period,base_position,base_orientation_);
}

void CommandsInterface::update(const double& period) // OpenLoop
{
    update(period,base_position_,base_orientation_);
}

void CommandsInterface::initializeFootPosition(const std::string& foot_name)
{
    xbot_model_->getPose(foot_name,world_T_foot_);
    gait_generator_->setInitialPose(foot_name,world_T_foot_);
}

void CommandsInterface::initializeFeetPosition()
{
    const std::vector<std::string>& feet_names = gait_generator_->getFeetNames();

    for(unsigned int i=0; i<feet_names.size(); i++)
        initializeFootPosition(feet_names[i]);
}

void CommandsInterface::update(const double& period, const Eigen::Vector3d& base_position, const Eigen::Vector3d& base_orientation) // ClosedLoop
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

    setHipOffset();

    switch(cmd)
    {

    case cmd_t::HOLD:
        resetBaseVelocities();
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
        if(gait_generator_->isCycleEnded(feet_names[i]))
            initializeFootPosition(feet_names[i]);

        gait_generator_->setStepLength(feet_names[i], steps_length_[feet_names[i]]);
        gait_generator_->setStepHeading(feet_names[i], steps_heading_[feet_names[i]]);
        gait_generator_->setStepHeight(feet_names[i], steps_height_[feet_names[i]]);
        gait_generator_->setStepHeadingRate(feet_names[i], steps_heading_rate_[feet_names[i]]);
    }

    // Update the gait_generator
    gait_generator_->update(period);
}

void CommandsInterface::calculateFeetStep()
{
    const std::vector<std::string>& feet_names = gait_generator_->getFeetNames();
    const std::vector<std::string>& hips_names = gait_generator_->getHipsNames();

    for(unsigned int i=0; i<feet_names.size(); i++)
    {
        if(gait_generator_->isSwinging(feet_names[i]))
        {
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"*********");
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"Swinging foot "<<feet_names[i]);

            xbot_model_->getPose(feet_names[i],world_T_foot_);
            xbot_model_->getPose(feet_names[i],"base_link",base_T_foot_);
            xbot_model_->getPose(hips_names[i],world_T_hip_);
            //xbot_model_->getPose(hips_names[i],"base_link",base_T_hip_);

            hf_delta_hip_.setZero();
            hf_delta_hip_(0) = hf_base_linear_velocity_(0)*1.0/gait_generator_->getSwingFrequency(feet_names[i]);
            hf_delta_hip_(1) = hf_base_linear_velocity_(1)*1.0/gait_generator_->getSwingFrequency(feet_names[i]);

            //hf_X_hip_ = hf_R_base_ * base_T_hip_.translation();
            hf_delta_heding_.setZero();
            hf_delta_heding_(2) = hf_base_angular_velocity_(2)*1.0/gait_generator_->getSwingFrequency(feet_names[i]);
            hf_delta_heding_ = hf_delta_heding_.cross(hf_X_virtual_hips_[i]);
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_delta_heding_: "<<hf_delta_heding_.transpose());

            hf_delta_hip_(0)+= hf_delta_heding_(0);
            hf_delta_hip_(1)+= hf_delta_heding_(1);
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"hf_delta_hip_(velocity based): "<<hf_delta_hip_.transpose());

            world_delta_hip_ = world_R_hf_ * hf_delta_hip_;
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_delta_hip_(velocity based): "<<world_delta_hip_.transpose());

            world_X_virtual_hip_ = world_R_hf_ *(hf_X_virtual_hips_[i] - hf_R_base_*base_T_foot_.translation() );
            world_X_virtual_hip_(2)=0;
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_X_virtual_hip_(distance of actual foot position wrt to virtual hip pos in wf): "<<world_X_virtual_hip_.transpose());

            world_X_hip_foot_offset_ = world_R_hf_ * hf_X_hip_foot_offsets_[i];
            world_X_hip_foot_offset_(2) = 0;
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_X_hip_foot_offset_: "<<world_X_hip_foot_offset_.transpose());

            world_delta_foot_.setZero();
            world_delta_foot_.head(2) =   world_delta_hip_.head(2)  + world_X_hip_foot_offset_.head(2) + world_X_virtual_hip_.head(2);

            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"world_delta_foot_: "<<world_delta_foot_.transpose());

            // if(std::abs(hf_base_linear_velocity_(0))!=0.0 || std::abs(hf_base_linear_velocity_(1))!=0.0)
            step_length_ = std::sqrt(world_delta_foot_(0)*world_delta_foot_(0) + world_delta_foot_(1)*world_delta_foot_(1));
            // else
            //    step_length_ = 0.0;
            //step_height_ = 0.05; // FIXME

            if(step_length_ > step_length_max_)
            {
                step_length_ = step_length_max_;
                ROS_WARN_STREAM_NAMED(CLASS_NAME,"Step length is greater than: "<<step_length_max_);
            }

            step_height_ = step_height_max_; // FIXME for the moment we set the step heigh at the max value
            if(step_height_ > step_height_max_)
            {
                step_height_ = step_height_max_;
                ROS_WARN_STREAM_NAMED(CLASS_NAME,"Step height is greater than: "<<step_height_max_);
            }

            steps_length_[feet_names[i]]         = step_length_;
            steps_heading_[feet_names[i]]        = std::atan2(world_delta_foot_(1),world_delta_foot_(0));
            steps_height_[feet_names[i]]         = step_height_;
            steps_heading_rate_[feet_names[i]]   = hf_base_angular_velocity_(2);

            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"steps_length["<<feet_names[i]<<"]: "<<steps_length_[feet_names[i]]);
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"steps_heading_["<<feet_names[i]<<"]: "<<steps_heading_[feet_names[i]]);
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"steps_height_["<<feet_names[i]<<"]: "<<steps_height_[feet_names[i]]);
            ROS_DEBUG_STREAM_NAMED(CLASS_NAME,"steps_heading_rate_["<<feet_names[i]<<"]: "<<steps_heading_rate_[feet_names[i]]);
        }
        else
        {
            steps_length_[feet_names[i]]         = 0.0;
            steps_heading_[feet_names[i]]        = 0.0;
            steps_height_[feet_names[i]]         = 0.0;
            steps_heading_rate_[feet_names[i]]   = 0.0;
        }
    }

    gait_generator_->activateSwing();
}

void CommandsInterface::resetFeetStep()
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

void CommandsInterface::resetBaseAngularVelocity()
{
    hf_base_angular_velocity_.setZero();
    hf_base_angular_velocity_ref_.setZero();
    hf_base_angular_velocity_filt_.setZero();
}

void CommandsInterface::resetBaseLinearVelocity()
{
    hf_base_linear_velocity_.setZero();
    hf_base_linear_velocity_ref_.setZero();
    hf_base_linear_velocity_filt_.setZero();
}

void CommandsInterface::resetBaseVelocities()
{
    resetBaseAngularVelocity();
    resetBaseLinearVelocity();
}

void CommandsInterface::resetBasePosition()
{
    for(unsigned int i=0;i<3;i++)
        base_position_(i) = secondOrderFilter(base_position_(i),base_position_filt_(i),default_base_position_(i),1.0);

    base_height_ = base_position_(2);
}

void CommandsInterface::resetBaseOrientation()
{
    default_base_orientation_(2) = base_orientation_(2); // Keep the same yaw

    for(unsigned int i=0;i<3;i++)
        base_orientation_(i) = secondOrderFilter(base_orientation_(i),base_orientation_filt_(i),default_base_orientation_(i),1.0); //FIXME hardcoded gain, it should be based on the sampling time

    rpyToRot(base_orientation_,base_rotation_reference_);
    base_rotation_reference_.transposeInPlace();
}

void CommandsInterface::calculateBasePosition(const double& period, const Eigen::Vector3d& base_position)
{
    base_position_ = base_position;

    hf_base_linear_velocity_ref_(0) = base_linear_velocity_max_ * base_linear_velocity_scale_x_;
    hf_base_linear_velocity_ref_(1) = base_linear_velocity_max_ * base_linear_velocity_scale_y_;
    hf_base_linear_velocity_ref_(2) = base_linear_velocity_max_ * base_linear_velocity_scale_z_;

    for(unsigned int i=0;i<3;i++)
        hf_base_linear_velocity_(i) = secondOrderFilter(hf_base_linear_velocity_(i),hf_base_linear_velocity_filt_(i),hf_base_linear_velocity_ref_(i),0.5); //FIXME hardcoded gain, it should be based on the sampling time

    base_position_ = world_R_hf_ * hf_base_linear_velocity_ * period + base_position_;

    // This is the base height computed w.r.t world
    base_height_ = base_position_(2);
}

void CommandsInterface::calculateBaseOrientation(const double& period, const Eigen::Vector3d& base_orientation)
{
    base_orientation_ = base_orientation;

    hf_base_angular_velocity_ref_(0) = base_angular_velocity_max_ * base_angular_velocity_scale_roll_;
    hf_base_angular_velocity_ref_(1) = base_angular_velocity_max_ * base_angular_velocity_scale_pitch_;
    hf_base_angular_velocity_ref_(2) = base_angular_velocity_max_ * base_angular_velocity_scale_yaw_;

    for(unsigned int i=0;i<3;i++)
        hf_base_angular_velocity_(i) = secondOrderFilter(hf_base_angular_velocity_(i),hf_base_angular_velocity_filt_(i),hf_base_angular_velocity_ref_(i),0.5);

    base_orientation_ = hf_base_angular_velocity_ * period + base_orientation_;

     // This is the base rotation computed w.r.t world
    rpyToRot(base_orientation_,base_rotation_reference_);
    base_rotation_reference_.transposeInPlace();
}

void CommandsInterface::setHipOffset()
{
    if(!offset_applied_)
    {
        const std::vector<std::string>& hips_names = gait_generator_->getHipsNames();
        for(unsigned int i=0; i<hips_names.size(); i++)
        {
            xbot_model_->getPose(gait_generator_->getFeetNames()[i],"base_link",base_T_foot_);
            xbot_model_->getPose(hips_names[i],"base_link",base_T_hip_);
            //initial feet offsets
            hf_X_hip_foot_offsets_[i] = hf_R_base_ * (base_T_foot_.translation() - base_T_hip_.translation());

            //virtual hips we assume base starts horizzontal (TODO)
            hf_X_virtual_hips_[i] = base_T_hip_.translation();

        }

        ROS_DEBUG_STREAM("The signs for hf_X_base_hip_offsets_[lf] are "
                         << hf_X_virtual_hips_[0] <<
                         " they should be: +,+ and 0.0");

        ROS_DEBUG_STREAM("The signs for hf_X_base_hip_offsets_[rf] are "
                         << hf_X_virtual_hips_[1] <<
                         " they should be: +,- and 0.0");

        ROS_DEBUG_STREAM("The signs for hf_X_base_hip_offsets_[lh] are "
                         << hf_X_virtual_hips_[2] <<
                         " they should be: -,+ and 0.0");

        ROS_DEBUG_STREAM("The signs for hf_X_base_hip_offsets_[rh] are "
                         << hf_X_virtual_hips_[3] <<
                         " they should be: -,- and 0.0");

        offset_applied_ = true;
    }
}

// Sets
void CommandsInterface::setCmd(const unsigned int cmd)
{
    cmd_ = cmd;
}

void CommandsInterface::setBasePosition(const Eigen::Vector3d& position)
{
    base_position_ = position;
}

void CommandsInterface::setBaseOrientation(const Eigen::Vector3d& orientation)
{
    base_orientation_ = orientation;
}

void CommandsInterface::setDefaultBaseOrientation(const Eigen::Vector3d& orientation)
{
    default_base_orientation_ = orientation;
}

void CommandsInterface::setDefaultBasePosition(const Eigen::Vector3d& position)
{
    default_base_position_ = position;
}

void CommandsInterface::setBaseVelocityScaleX(const double scale)
{
    base_linear_velocity_scale_x_ = scale;
}

void CommandsInterface::setBaseVelocityScaleY(const double scale)
{
    base_linear_velocity_scale_y_ = scale;
}

void CommandsInterface::setBaseVelocityScaleZ(const double scale)
{
    base_linear_velocity_scale_z_ = scale;
}

void CommandsInterface::setBaseVelocityScaleRoll(const double scale)
{
    base_angular_velocity_scale_roll_ = scale;
}

void CommandsInterface::setBaseVelocityScalePitch(const double scale)
{
    base_angular_velocity_scale_pitch_ = scale;
}

void CommandsInterface::setBaseVelocityScaleYaw(const double scale)
{
    base_angular_velocity_scale_yaw_ = scale;
}

void CommandsInterface::setMaxLinearVelocity(const double max)
{
    base_linear_velocity_max_ = max;
}

void CommandsInterface::setMaxAngularVelocity(const double max)
{
    base_angular_velocity_max_ = max;
}

void CommandsInterface::setMaxStepHeight(const double max)
{
    step_height_max_ = max;
}

void CommandsInterface::setMaxStepLength(const double max)
{
    step_length_max_ = max;
}

// Gets
unsigned int CommandsInterface::getCmd()
{
    return cmd_;
}

const Eigen::Matrix3d& CommandsInterface::getBaseRotationReference() const
{
    return base_rotation_reference_;
}

const double& CommandsInterface::getStepLength(const std::string& foot_name)
{
    return steps_length_[foot_name];
}

const double& CommandsInterface::getStepHeading(const std::string& foot_name)
{
    return steps_heading_[foot_name];
}

const double& CommandsInterface::getStepHeight(const std::string& foot_name)
{
    return steps_height_[foot_name];
}

const double& CommandsInterface::getStepHeadingRate(const std::string& foot_name)
{
    return steps_heading_rate_[foot_name];
}

const double& CommandsInterface::getBaseHeight() const
{
    return base_height_;
}

double CommandsInterface::getMaxLinearVelocity() const
{
    return base_linear_velocity_max_;
}

double CommandsInterface::getMaxAngularVelocity() const
{
    return base_angular_velocity_max_;
}

double CommandsInterface::getMaxStepHeight() const
{
    return step_height_max_;
}

double CommandsInterface::getMaxStepLength() const
{
    return step_length_max_;
}

}; // namespace
