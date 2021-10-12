#include "wb_controller/legs_impedance.h"

namespace wb_controller {

LegsImpedance::LegsImpedance(GaitGenerator::Ptr gait_generator, QuadrupedRobot::Ptr robot_model)
{
    robot_model_ = robot_model;
    gait_generator_ = gait_generator;
    inertia_compensation_active_ = true;

    // Initialize the inertia related matrices
    robot_model_->getInertiaMatrix(M_);
    Mi_.setZero(3,3);
    Kp_postural_.setIdentity(M_.rows(), M_.cols());
    Kd_postural_.setIdentity(M_.rows(), M_.cols());
}

void LegsImpedance::startInertiaCompensation(const bool& start)
{
    inertia_compensation_active_ = start;
}

void LegsImpedance::update(const Eigen::Matrix3d& Kp_swing_leg, const Eigen::Matrix3d& Kp_stance_leg,
                           const Eigen::Matrix3d& Kd_swing_leg, const Eigen::Matrix3d& Kd_stance_leg)
{
    const std::vector<std::string>& foot_names = robot_model_->getFootNames();
    const std::vector<std::string>& leg_names  = robot_model_->getLegNames();

    for(unsigned int i=0;i<foot_names.size();i++)
    {
        int idx = robot_model_->getLimbJointsIds(leg_names[i])[0]; // NOTE: take the first idx, hopefully the leg joints are contiguos

        if(gait_generator_->isSwinging(foot_names[i]))
        {
            if(inertia_compensation_active_)
            {
                robot_model_->getLimbInertiaInverse(leg_names[i],Mi_);
                Kp_postural_.block<3,3>(idx,idx) = Mi_ * Kp_swing_leg;
                Kd_postural_.block<3,3>(idx,idx) = Mi_ * Kd_swing_leg;
            }
            else
            {
                Kp_postural_.block<3,3>(idx,idx) = Kp_swing_leg;
                Kd_postural_.block<3,3>(idx,idx) = Kd_swing_leg;
            }
        }
        else
        {
            Kp_postural_.block<3,3>(idx,idx) = Kp_stance_leg;
            Kd_postural_.block<3,3>(idx,idx) = Kd_stance_leg;
        }
    }
}

const Eigen::MatrixXd& LegsImpedance::getKp() const
{
    return Kp_postural_;
}

const Eigen::MatrixXd& LegsImpedance::getKd() const
{
    return Kd_postural_;
}

} // namespace
