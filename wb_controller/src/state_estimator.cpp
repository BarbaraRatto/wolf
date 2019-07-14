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

    imu_reset_done_ = false;
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

    Eigen::Matrix3d Ear, base_R_world;
    Eigen::Vector3d raw_base_rpy;

   // NOTE: Check if the imu is in the correct frame, here we assume that it is aligned with the trunk/bas

   //tmp_matrix3d_ = world_R_imu_init_.transpose() * tmp_matrix3d_; // imu_R_world = R_imu0' * R_imu

   quatToRotMat(imu_orientation_.normalized(),tmp_matrix3d_);
   rotTorpy(tmp_matrix3d_,raw_base_rpy);

   // Reset the imu one time so that the world and the base are aligned
   if(!imu_reset_done_)
   {
       base_rpy_ = raw_base_rpy;
       imu_reset_done_ = true;
   }

   //quatToRpy(floating_base_orientation_.normalized(),floating_base_orientation_rpy_);//take rpy measures

   //rpyToEarInv(base_rpy_,Ear);
   rpyToEar(base_rpy_,Ear);
   //intergate gyro
   base_rpy_ += (Ear.inverse() * imu_gyroscope_)*period; //maps base_omega into rpyderivatives
   //overwrite measures
   base_rpy_.head(2) = raw_base_rpy.head(2);

   rpyToRot(base_rpy_, base_R_world);
   floating_base_pose_.linear() = base_R_world.transpose();
   floating_base_velocity_.segment(3,3) = base_R_world.transpose() * imu_gyroscope_;

   xbot_model_->setJointVelocity(joint_velocities_);
   xbot_model_->setJointEffort(joint_efforts_);
   xbot_model_->setJointPosition(joint_positions_);
   xbot_model_->setFloatingBaseOrientation(floating_base_pose_.linear());
   xbot_model_->setFloatingBaseAngularVelocity(floating_base_velocity_.segment(3,3));
   xbot_model_->update();

   qp_estimation_->update();
   qp_estimation_->getFloatingBaseTwist(floating_base_velocity_qp_);

   floating_base_velocity_.segment(0,3) << 0.0,0.0,floating_base_velocity_qp_(2);
   //floating_base_velocity_.segment(0,3) = floating_base_velocity_qp_.segment(0,3);
   //floating_base_velocity_.segment(3,3) = imu_gyroscope_;

   // Estimate z
   double estimated_z = 0;
   int feet_in_stance = 0;
   for(unsigned int i = 0; i<feet_names.size(); i++)
   {
       if(!gait_generator_->isSwinging(feet_names[i]))
       {
           xbot_model_->getPose(feet_names[i],"base_link",tmp_affine3d_);
           feet_in_stance++;

           tmp_affine3d_.translation() = tmp_matrix3d_.transpose() *  tmp_affine3d_.translation();

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
