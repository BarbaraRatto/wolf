#include <wb_controller/state_estimator.h>
#include <wb_controller/utils.h>

namespace wb_controller {

#define CLASS_NAME "StateEstimator"

StateEstimator::StateEstimator(GaitGenerator::Ptr gait_generator, XBot::ModelInterface::Ptr xbot_model)
{

    assert(xbot_model);
    xbot_model_ = xbot_model;

    assert(gait_generator);
    gait_generator_ = gait_generator;

    const std::vector<std::string>& feet_names = gait_generator_->getFeetNames();

    // Floating Base state estimation reset
    Eigen::Matrix6d contact_matrix;
    contact_matrix.setZero();
    contact_matrix.block(0,0,3,3) << Eigen::Matrix3d::Identity();
    qp_estimation_.reset(new OpenSoT::floating_base_estimation::qp_estimation(xbot_model_,feet_names,contact_matrix));

    int n_dofs = xbot_model_->getJointNum();
    joint_positions_.resize(static_cast<Eigen::Index>(n_dofs));
    joint_velocities_.resize(static_cast<Eigen::Index>(n_dofs));
    joint_efforts_.resize(static_cast<Eigen::Index>(n_dofs));
    base_rpy_ = Eigen::Vector3d::Zero();
    floating_base_position_ = Eigen::Vector3d::Zero();
    floating_base_velocity_ = Eigen::Vector6d::Zero();
    floating_base_pose_ = Eigen::Affine3d::Identity();
    terrain_normal_ << 0,0,1; // TODO terrain estimation
    imu_orientation_.normalize();
    floating_base_velocity_qp_.resize(FLOATING_BASE_DOFS);
    contacts_.resize(N_LEGS);
    contact_forces_.resize(N_LEGS);
    world_X_foot_.resize(N_LEGS);
    base_X_foot_.resize(N_LEGS);
    world_T_foot_.resize(N_LEGS);
    base_T_foot_.resize(N_LEGS);

    contacts_estimation_active_ = false;

    for(unsigned int i=0;i<feet_names.size();i++)
    {
        contacts_[i] = true;
        contact_forces_[i] = Eigen::Vector3d::Zero();
        world_X_foot_[i] = Eigen::Vector3d::Zero();
        base_X_foot_[i] = Eigen::Vector3d::Zero();
        world_T_foot_[i] = Eigen::Affine3d::Identity();
        base_T_foot_[i] = Eigen::Affine3d::Identity();
    }

    Ear_ = Eigen::Matrix3d::Identity();
    base_R_world_ = Eigen::Matrix3d::Identity();

    estimation_orientation_ = estimation_t::IMU_MAGNETOMETER;
    estimation_position_ = estimation_t::ESTIMATED_Z;

    imu_reset_done_ = false;

    haptic_contact_loop_active_ = false;

    contact_force_th_ = 0.0; // [N]

    // Contact force estimation reset
    force_estimation_.reset(new XBot::Cartesian::Utils::ForceEstimation(xbot_model_));

    // Contact estimation reset, FIXME to clean up and add the ARM
    std::vector<int> dofs = {0,1,2}; // x y z
    std::vector<std::string> chains, feet_chains;
    std::vector<std::string> chain(1);
    chains = xbot_model_->getChainNames();

    for(unsigned int i = 0; i < chains.size(); i++) // Remove virtual_chain and arm
        if(chains[i].find("arm") == std::string::npos && chains[i].find("virtual_chain") == std::string::npos)
            feet_chains.push_back(chains[i]);

    assert(feet_chains.size() == N_LEGS);
    feet_chains = sortByLegName(feet_chains);
    for(unsigned int i=0;i<feet_names.size();i++)
    {
        chain[0] = feet_chains[i];
        force_torque_sensors_.push_back(force_estimation_->add_link(feet_names[i],dofs,chain));
    }

    Logger::getLogger().addPublisher(CLASS_NAME"/floating_base_position",floating_base_position_);
    Logger::getLogger().addPublisher(CLASS_NAME"/floating_base_velocity",floating_base_velocity_);
    Logger::getLogger().addPublisher(CLASS_NAME"/floating_base_velocity_qp",floating_base_velocity_qp_);
    Logger::getLogger().addPublisher(CLASS_NAME"/base_rpy",base_rpy_);
}

void StateEstimator::setEstimationType(unsigned int position_t, unsigned int orientation_t)
{
    estimation_orientation_ = orientation_t;
    estimation_position_ = position_t;
}

void StateEstimator::setPositionEstimationType(unsigned int position_t)
{
    estimation_position_ = position_t;
}

void StateEstimator::setOrientationEstimationType(unsigned int orientation_t)
{
    estimation_orientation_ = orientation_t;
}

void StateEstimator::setJointPosition(const Eigen::VectorXd& joint_positions)
{
    joint_positions_ = joint_positions;
}

void StateEstimator::setJointVelocity(const Eigen::VectorXd& joint_velocities)
{
    joint_velocities_ = joint_velocities;
}

void StateEstimator::setJointEffort(const Eigen::VectorXd& joint_efforts)
{
    joint_efforts_ = joint_efforts;
}

void StateEstimator::setImuOrientation(const Eigen::Quaterniond& imu_orientation)
{
    imu_orientation_ = imu_orientation;
}

void StateEstimator::setImuGyroscope(const Eigen::Vector3d& imu_gyroscope)
{
    imu_gyroscope_ = imu_gyroscope;
}

void StateEstimator::setGroundTruthBasePosition(const Eigen::Vector3d& gt_position)
{
    gt_position_ = gt_position;
}

void StateEstimator::setGroundTruthBaseOrientation(const Eigen::Quaterniond& gt_orientation)
{
    gt_orientation_ = gt_orientation;
}

void StateEstimator::setGroundTruthBaseLinearVelocity(const Eigen::Vector3d& gt_linear_velocity)
{
    gt_linear_velocity_ = gt_linear_velocity;
}

void StateEstimator::setGroundTruthBaseAngularVelocity(const Eigen::Vector3d& gt_angular_velocity)
{
    gt_angular_velocity_ = gt_angular_velocity;
}

void StateEstimator::setContactThreshold(const double& th)
{
    contact_force_th_ = th;
}

double StateEstimator::getContactThreshold()
{
    return contact_force_th_;
}

unsigned int StateEstimator::getPositionEstimationType()
{
    return estimation_position_;
}

unsigned int StateEstimator::getOrientationEstimationType()
{
    return estimation_orientation_;
}

const Eigen::Affine3d& StateEstimator::getFloatingBasePose() const
{
    return floating_base_pose_;
}

const Eigen::Vector3d& StateEstimator::getFloatingBasePosition() const
{
    return floating_base_position_;
}

const Eigen::Vector3d& StateEstimator::getFloatingBaseOrientationRPY() const
{
    return base_rpy_;
}

const std::vector<Eigen::Vector3d>& StateEstimator::getContactForces() const
{
    return contact_forces_;
}

const std::vector<bool>& StateEstimator::getContacts() const
{
    return contacts_;
}

const std::vector<Eigen::Vector3d>& StateEstimator::getFeetPositionInWorld() const
{
    return world_X_foot_;
}

const std::vector<Eigen::Vector3d>& StateEstimator::getFeetPositionInBase() const
{
    return base_X_foot_;
}

const std::vector<Eigen::Affine3d>& StateEstimator::getFeetPoseInWorld() const
{
    return world_T_foot_;
}

const std::vector<Eigen::Affine3d>& StateEstimator::getFeetPoseInBase() const
{
    return base_T_foot_;
}

void StateEstimator::toggleHapticContactLoop()
{
    haptic_contact_loop_active_=!haptic_contact_loop_active_;
}

void StateEstimator::startContactsEstimation()
{
    contacts_estimation_active_ = true;

    ROS_INFO("Start contact estimation");

}

void StateEstimator::stopContactsEstimation()
{
    contacts_estimation_active_ = false;

    ROS_INFO("Stop contact estimation: the contacts state are forced to TRUE");
}

void StateEstimator::update(const double& period)
{
    const std::vector<std::string>& feet_names = gait_generator_->getFeetNames();

    for(unsigned int i=0; i<feet_names.size(); i++)
    {
        // Feet position in world
        xbot_model_->getPose(feet_names[i],world_T_foot_[i]);
        world_X_foot_[i] = world_T_foot_[i].translation();
        // Feet position in base/trunk
        xbot_model_->getPose(feet_names[i],"base_link",base_T_foot_[i]);
        base_X_foot_[i] = base_T_foot_[i].translation();
    }

    updateContactState();

    updateFloatingBase(period);
}

void StateEstimator::updateContactState()
{

    const std::vector<std::string>& feet_names = gait_generator_->getFeetNames();

    force_estimation_->update();

    for(unsigned int i=0; i<contacts_.size(); i++)
    {
        force_torque_sensors_[i]->getForce(tmp_vector3d_); // tmp_vector3d_ = contact_force_foot

        tmp_vector3d_ = world_T_foot_[i] * tmp_vector3d_; // contact_force_world = world_T_foot * contact_force_foot

        if(contacts_estimation_active_)
            contacts_[i] = (tmp_vector3d_.dot(terrain_normal_) >= contact_force_th_ ? true : false);
        else
            contacts_[i] = true;

        contact_forces_[i] = tmp_vector3d_;

        if(haptic_contact_loop_active_)
        {
            qp_estimation_->setContactState(feet_names[i],contacts_[i]);
            gait_generator_->setContactState(feet_names[i],contacts_[i]);
        }
        else
        {
            qp_estimation_->setContactState(feet_names[i],gait_generator_->isTrajectoryFinished(feet_names[i]));
            gait_generator_->setContactState(feet_names[i],false);
        }
    }
}

void StateEstimator::updateFloatingBase(const double& period)
{
    unsigned int estimation_orientation = estimation_orientation_;
    unsigned int estimation_position = estimation_position_;

    // Update the joints information of the virtual model
    xbot_model_->setJointVelocity(joint_velocities_);
    xbot_model_->setJointEffort(joint_efforts_);
    xbot_model_->setJointPosition(joint_positions_);

    // Note: we assume that the IMU is orientated as the base/waist of the robot
    // if this is not the case, it is necessary to add a transfomation from the IMU frame to the
    // base/waist frame

    switch(estimation_orientation)
    {
    case estimation_t::NONE:
        // The base does not rotate
        floating_base_pose_.linear() = Eigen::Matrix3d::Identity();
        floating_base_velocity_.segment(3,3) << 0.0, 0.0, 0.0;
        break;
    case estimation_t::IMU_MAGNETOMETER: // Use directly the orientation information from the IMU
        quatToRotMat(imu_orientation_.normalized(),base_R_world_);
        rotTorpy(base_R_world_,base_rpy_);
        floating_base_pose_.linear() = base_R_world_.transpose();
        //floating_base_velocity_.segment(3,3) = floating_base_pose_.linear() * imu_gyroscope_; // This line is making the robot crash
        floating_base_velocity_.segment(3,3) = imu_gyroscope_;
        break;
    case estimation_t::IMU_GYROSCOPE: // Intergate the gyroscope, useful if the magnetometer measure has interferences
        if(!imu_reset_done_)
        {
            // Initialization for the integration
            quatToRotMat(imu_orientation_.normalized(),base_R_world_);
            rotTorpy(base_R_world_,base_rpy_);
            imu_reset_done_ = true;
        }
        rpyToEarInv(base_rpy_,Ear_);
        // Map the omegas in the base into rpy derivatives
        base_rpy_ += (Ear_.inverse() * imu_gyroscope_) * period;
        // Overwrite measures if one of them is more noisy
        // base_rpy_.head(2) = raw_base_rpy_.head(2);
        rpyToRot(base_rpy_, base_R_world_);
        floating_base_pose_.linear() = base_R_world_.transpose();
        //floating_base_velocity_.segment(3,3) = floating_base_pose_.linear() * imu_gyroscope_; // This line is making the robot crash
        floating_base_velocity_.segment(3,3) = imu_gyroscope_;
        break;
    case estimation_t::GROUND_TRUTH:
        quatToRotMat(gt_orientation_.normalized(),base_R_world_);
        rotTorpy(base_R_world_,base_rpy_);
        floating_base_pose_.linear() = base_R_world_.transpose();
        floating_base_velocity_.segment(3,3) = floating_base_pose_.linear() * gt_angular_velocity_;
        break;
    default:
        // The base does not rotate
        floating_base_pose_.linear() = Eigen::Matrix3d::Identity();
        floating_base_velocity_.segment(3,3) << 0.0, 0.0, 0.0;
        break;
    };

    //// Reset the imu one time so that the world and the base are aligned
    ////tmp_matrix3d_ = world_R_imu_init_.transpose() * tmp_matrix3d_; // imu_R_world = R_imu0' * R_imu
    ////quatToRpy(floating_base_orientation_.normalized(),floating_base_orientation_rpy_);//take rpy measures

    // Update the orientation part of the floating base
    // This is necessary if we are going to use the qp estimator for the floating base linear velocities
    xbot_model_->setFloatingBaseOrientation(floating_base_pose_.linear());
    xbot_model_->setFloatingBaseAngularVelocity(floating_base_velocity_.segment(3,3));
    xbot_model_->update();

    const std::vector<std::string>& feet_names = gait_generator_->getFeetNames();

    double estimated_z = 0.0;
    int feet_in_stance = 0;

    switch(estimation_position)
    {
    case estimation_t::NONE:
        // The base does not move
        floating_base_velocity_.segment(0,3) << 0.0,0.0,0.0;
        floating_base_position_ << 0.0,0.0,0.0;
        break;
    case estimation_t::ESTIMATED_Z:
        // Estimate z using the legs position
        for(unsigned int i = 0; i<feet_names.size(); i++)
        {
            if(!gait_generator_->isSwinging(feet_names[i]))
            {
                feet_in_stance++;
                tmp_affine3d_.translation() = floating_base_pose_.linear() *  base_X_foot_[i];
                estimated_z +=  tmp_affine3d_.translation().z();
            }
        }
        estimated_z /= feet_in_stance;
        // Update the qp estimation based on the new virtual model state
        qp_estimation_->update();
        qp_estimation_->getFloatingBaseTwist(floating_base_velocity_qp_);
        floating_base_velocity_.segment(0,3) << 0.0,0.0,0.0; // Does not work, the robot fall!
        //floating_base_velocity_.segment(0,3) << 0.0,0.0,floating_base_velocity_qp_(2); // Does not work, the robot fall!
        //floating_base_velocity_.segment(0,3) = floating_base_velocity_qp_.segment(0,3);
        floating_base_position_ << 0.0,0.0, -estimated_z; // Remove x and y from the state estimation
        break;
    case estimation_t::GROUND_TRUTH:
        floating_base_velocity_.segment(0,3) << gt_linear_velocity_;
        floating_base_position_ << gt_position_;
        break;
    default:
        // The base does not move
        floating_base_velocity_.segment(0,3) << 0.0,0.0,0.0;
        floating_base_position_ << 0.0,0.0,0.0;
        break;
    };


    // Finally update the floating base with the full pose and velocities
    floating_base_pose_.translation() = floating_base_position_;
    xbot_model_->setFloatingBaseState(floating_base_pose_,floating_base_velocity_); // This should trigger the update of the model
}

}
