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

    // Floating Base state estimation reset
    Eigen::Matrix6d contact_matrix;
    contact_matrix.setZero();
    contact_matrix.block(0,0,3,3) << Eigen::Matrix3d::Identity();
    qp_estimation_.reset(new OpenSoT::floating_base_estimation::qp_estimation(xbot_model_,gait_generator_->getFeetNames(),contact_matrix));

    int n_dofs = xbot_model_->getJointNum();
    joint_positions_.resize(static_cast<Eigen::Index>(n_dofs));
    joint_velocities_.resize(static_cast<Eigen::Index>(n_dofs));
    joint_efforts_.resize(static_cast<Eigen::Index>(n_dofs));
    base_rpy_ = Eigen::Vector3d::Zero();
    floating_base_position_ = Eigen::Vector3d::Zero();
    floating_base_velocity_ = Eigen::Vector6d::Zero();
    floating_base_pose_ = Eigen::Affine3d::Identity();
    imu_orientation_.normalize();
    floating_base_velocity_qp_.resize(FLOATING_BASE_DOFS);

    Ear_ = Eigen::Matrix3d::Identity();
    base_R_world_ = Eigen::Matrix3d::Identity();

    estimation_ = estimation_t::IMU_MAGNETOMETER;

    imu_reset_done_ = false;

    Logger::getLogger().addPublisher(CLASS_NAME"/floating_base_position",floating_base_position_);
    Logger::getLogger().addPublisher(CLASS_NAME"/floating_base_velocity",floating_base_velocity_);
    Logger::getLogger().addPublisher(CLASS_NAME"/floating_base_velocity_qp_",floating_base_velocity_qp_);
    Logger::getLogger().addPublisher(CLASS_NAME"/base_rpy",base_rpy_);
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

void StateEstimator::setContactState(const std::string& foot_name, const bool& contact_state)
{
    if(qp_estimation_)
        qp_estimation_->setContactState(foot_name,contact_state);
    else
        ROS_WARN_NAMED(CLASS_NAME,"qp estimator not initialized yet.");
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

void StateEstimator::update(const double& period)
{
    const std::vector<std::string>& feet_names = gait_generator_->getFeetNames();

    unsigned int estimation = estimation_;

    // Note: we assume that the IMU is orientated as the base/waist of the robot
    // if this is not the case, it is necessary to add a transfomation from the IMU frame to the
    // base/waist frame

    switch(estimation)
    {
    case estimation_t::IMU_MAGNETOMETER:
        quatToRotMat(imu_orientation_.normalized(),base_R_world_);
        rotTorpy(base_R_world_,base_rpy_);
        break;
        // Intergate the gyroscope, useful if the magnetometer measure has interferences
    case estimation_t::IMU_GYROSCOPE:
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
        break;
    default:
        break;
    };

    //// Reset the imu one time so that the world and the base are aligned
    ////tmp_matrix3d_ = world_R_imu_init_.transpose() * tmp_matrix3d_; // imu_R_world = R_imu0' * R_imu
    ////quatToRpy(floating_base_orientation_.normalized(),floating_base_orientation_rpy_);//take rpy measures

    // Set the floating base orientation
    floating_base_pose_.linear() = base_R_world_.transpose();
    // Set the floating base angular velocity
    floating_base_velocity_.segment(3,3) = floating_base_pose_.linear() * imu_gyroscope_;
    // Update the virtual model
    xbot_model_->setJointVelocity(joint_velocities_);
    xbot_model_->setJointEffort(joint_efforts_);
    xbot_model_->setJointPosition(joint_positions_);
    xbot_model_->setFloatingBaseOrientation(floating_base_pose_.linear());
    xbot_model_->setFloatingBaseAngularVelocity(floating_base_velocity_.segment(3,3));
    xbot_model_->update();
    // Update the qp estimation based on the new virtual model state
    qp_estimation_->update();
    qp_estimation_->getFloatingBaseTwist(floating_base_velocity_qp_);

    //floating_base_velocity_.segment(0,3) << 0.0,0.0,floating_base_velocity_qp_(2);
    floating_base_velocity_.segment(0,3) = floating_base_velocity_qp_.segment(0,3);
    //floating_base_velocity_.segment(0,3) << 0.0,0.0,0.0;

    // Estimate z
    double estimated_z = 0;
    int feet_in_stance = 0;
    for(unsigned int i = 0; i<feet_names.size(); i++)
    {
        if(!gait_generator_->isSwinging(feet_names[i]))
        {
            xbot_model_->getPose(feet_names[i],"base_link",tmp_affine3d_); // world_T_foot
            feet_in_stance++;

            tmp_affine3d_.translation() = floating_base_pose_.linear() *  tmp_affine3d_.translation();

            estimated_z +=  tmp_affine3d_.translation().z();
        }
    }
    estimated_z /= feet_in_stance;

    floating_base_position_ << 0.0,0.0, -estimated_z; // Remove x and y from the state estimation
    //floating_base_position_ << 0.0,0.0,0.0; // Remove x y and z from the state estimation

    floating_base_pose_.translation() = floating_base_position_;

    xbot_model_->setFloatingBaseState(floating_base_pose_,floating_base_velocity_);

}

}
