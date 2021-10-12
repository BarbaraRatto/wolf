#include "wb_controller/legs_impedance.h"

namespace wb_controller {

void LegsImpedance::loadMatrices()
{
    Kp_swing_leg_(0,0) = kp_swing_haa_;
    Kp_swing_leg_(1,1) = kp_swing_hfe_;
    Kp_swing_leg_(2,2) = kp_swing_kfe_;
    Kd_swing_leg_(0,0) = kd_swing_haa_;
    Kd_swing_leg_(1,1) = kd_swing_hfe_;
    Kd_swing_leg_(2,2) = kd_swing_kfe_;

    Kp_stance_leg_(0,0) = kp_stance_haa_;
    Kp_stance_leg_(1,1) = kp_stance_hfe_;
    Kp_stance_leg_(2,2) = kp_stance_kfe_;
    Kd_stance_leg_(0,0) = kd_stance_haa_;
    Kd_stance_leg_(1,1) = kd_stance_hfe_;
    Kd_stance_leg_(2,2) = kd_stance_kfe_;
}

void LegsImpedance::loadValues()
{
    kp_swing_haa_  = Kp_swing_leg_(0,0);
    kp_swing_hfe_  = Kp_swing_leg_(1,1);
    kp_swing_kfe_  = Kp_swing_leg_(2,2);
    kd_swing_haa_  = Kd_swing_leg_(0,0);
    kd_swing_hfe_  = Kd_swing_leg_(1,1);
    kd_swing_kfe_  = Kd_swing_leg_(2,2);

    kp_stance_haa_ = Kp_stance_leg_(0,0);
    kp_stance_hfe_ = Kp_stance_leg_(1,1);
    kp_stance_kfe_ = Kp_stance_leg_(2,2);
    kd_stance_haa_ = Kd_stance_leg_(0,0);
    kd_stance_hfe_ = Kd_stance_leg_(1,1);
    kd_stance_kfe_ = Kd_stance_leg_(2,2);
}

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

    Kp_swing_leg_.setZero();
    Kd_swing_leg_.setZero();
    Kp_stance_leg_.setZero();
    Kd_stance_leg_.setZero();
}

void LegsImpedance::startInertiaCompensation(const bool& start)
{
    inertia_compensation_active_ = start;
}

void LegsImpedance::setSwingStanceGains(const Eigen::Vector3d& Kp_swing_leg, const Eigen::Vector3d& Kd_swing_leg,
                                        const Eigen::Vector3d& Kp_stance_leg, const Eigen::Vector3d& Kd_stance_leg)
{
    Kp_swing_leg_  = Kp_swing_leg.asDiagonal();
    Kd_swing_leg_  = Kd_swing_leg.asDiagonal();
    Kp_stance_leg_ = Kp_stance_leg.asDiagonal();
    Kd_stance_leg_ = Kd_stance_leg.asDiagonal();

    loadValues();
}

void LegsImpedance::update()
{
    const std::vector<std::string>& foot_names = robot_model_->getFootNames();
    const std::vector<std::string>& leg_names  = robot_model_->getLegNames();

    loadMatrices();

    for(unsigned int i=0;i<foot_names.size();i++)
    {
        int idx = robot_model_->getLimbJointsIds(leg_names[i])[0]; // NOTE: take the first idx, hopefully the leg joints are contiguos

        if(gait_generator_->isSwinging(foot_names[i]))
        {
            if(inertia_compensation_active_)
            {
                robot_model_->getLimbInertiaInverse(leg_names[i],Mi_);
                Kp_postural_.block<3,3>(idx,idx) = Mi_ * Kp_swing_leg_;
                Kd_postural_.block<3,3>(idx,idx) = Mi_ * Kd_swing_leg_;
            }
            else
            {
                Kp_postural_.block<3,3>(idx,idx) = Kp_swing_leg_;
                Kd_postural_.block<3,3>(idx,idx) = Kd_swing_leg_;
            }
        }
        else
        {
            Kp_postural_.block<3,3>(idx,idx) = Kp_stance_leg_;
            Kd_postural_.block<3,3>(idx,idx) = Kd_stance_leg_;
        }
    }
}

void LegsImpedance::setKpSwingLegHAA(const double &value)
{
    assert(value>=0.0);
    kp_swing_haa_=value;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set Kp swing HAA: "<<value);
}

void LegsImpedance::setKpSwingLegHFE(const double &value)
{
    assert(value>=0.0);
    kp_swing_hfe_=value;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set Kp swing HFE: "<<value);
}

void LegsImpedance::setKpSwingLegKFE(const double &value)
{
    assert(value>=0.0);
    kp_swing_kfe_=value;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set Kp swing KFE: "<<value);
}

void LegsImpedance::setKdSwingLegHAA(const double &value)
{
    assert(value>=0.0);
    kd_swing_haa_=value;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set Kd swing HAA: "<<value);
}

void LegsImpedance::setKdSwingLegHFE(const double &value)
{
    assert(value>=0.0);
    kd_swing_hfe_=value;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set Kd swing HFE: "<<value);
}

void LegsImpedance::setKdSwingLegKFE(const double &value)
{
    assert(value>=0.0);
    kd_swing_kfe_=value;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set Kd swing KFE: "<<value);
}

void LegsImpedance::setKpStanceLegHAA(const double &value)
{
    assert(value>=0.0);
    kp_stance_haa_=value;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set Kp stance HAA: "<<value);
}

void LegsImpedance::setKpStanceLegHFE(const double &value)
{
    assert(value>=0.0);
    kp_stance_hfe_=value;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set Kp stance HFE: "<<value);
}

void LegsImpedance::setKpStanceLegKFE(const double &value)
{
    assert(value>=0.0);
    kp_stance_kfe_=value;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set Kp stance KFE: "<<value);
}

void LegsImpedance::setKdStanceLegHAA(const double &value)
{
    assert(value>=0.0);
    kd_stance_haa_=value;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set Kd stance HAA: "<<value);
}

void LegsImpedance::setKdStanceLegHFE(const double &value)
{
    assert(value>=0.0);
    kd_stance_hfe_=value;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set Kd stance HFE: "<<value);
}

void LegsImpedance::setKdStanceLegKFE(const double &value)
{
    assert(value>=0.0);
    kd_stance_kfe_=value;
    ROS_INFO_STREAM_NAMED(CLASS_NAME,"Set Kd stance KFE: "<<value);
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
