#ifndef JOY_H
#define JOY_H

#include <ros/ros.h>
#include <atomic>
#include <sensor_msgs/Joy.h>

class JoyHandler
{

public:

    JoyHandler(ros::NodeHandle& node)
    {
        joy_foot_forward_scale_     = 0.0;
        joy_foot_lateral_scale_     = 0.0;
        joy_base_yaw_scale_         = 0.0;
        joy_base_pitch_scale_       = 0.0;
        joy_start_button_           = false;

        joy_sub_ = node.subscribe("joy", 1, &JoyHandler::joyCallback, this);
    }

    bool start() {return joy_start_button_;}
    double getFeetRotation() {return feet_rotation_;}



private:

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        joy_foot_lateral_scale_     = static_cast<double>(msg->axes[0]);
        joy_foot_forward_scale_     = static_cast<double>(msg->axes[1]);

        joy_base_yaw_scale_         = static_cast<double>(msg->axes[2]);
        joy_base_pitch_scale_       = static_cast<double>(msg->axes[3]);

        joy_start_button_           = static_cast<bool>(msg->buttons[4]); // L1 button

        // Set the joypad commands
       if(std::abs(joy_foot_forward_scale_)>0 || std::abs(joy_foot_lateral_scale_)>0) // Move the feet
        {
            //trj_yaw_ = std::atan2(joy_foot_lateral_scale_,joy_foot_forward_scale_);

            //gait_generator_->setTrajectoriesAmplitudes(trj_x_amp_,
            //                                           trj_yaw_,
            //                                           trj_z_amp_);

            feet_rotation_ = std::atan2(joy_foot_lateral_scale_,joy_foot_forward_scale_);
        }
        /*else if(std::abs(joy_base_yaw_scale_)>0)
        {

            yaw_rate_ = 0.05 * joy_base_yaw_scale_;





            Eigen::Vector3d angular_vel;
            Eigen::Vector3d delta_hip, delta_foot, delta_foot_world;
            Eigen::Affine3d world_T_foot, world_T_hip, world_T_base;
            Eigen::Matrix3d waist_rotation_reference;

            yaw_dot_ = 0.05 * joy_base_yaw_scale_; //rad/sec

            yaw_ = yaw_dot_*period.toSec() + yaw_;

            angular_vel << 0, 0, yaw_dot_;

            waist_rotation_reference = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());

            id_prob_->_waist->getReference(world_T_base);

            world_T_base.linear() = waist_rotation_reference;

            id_prob_->_waist->setReference(world_T_base);

            std::vector<std::string> hips(4);

            hips[0] = "lf_hipassembly";
            hips[1] = "rf_hipassembly";
            hips[2] = "lh_hipassembly";
            hips[3] = "rh_hipassembly";

            xbot_model_->getPose("base_link",world_T_base);

            for(unsigned int i=0; i<contact_links_.size(); i++)
            {

                xbot_model_->getPose(contact_links_[i],world_T_foot);
                xbot_model_->getPose(hips[i],world_T_hip);

                delta_hip = angular_vel.cross(world_T_hip.translation()); // It should be done in the world frame

                delta_foot = delta_hip + (world_T_hip.translation() - world_T_foot.translation());

                //delta_foot_world = world_T_base * delta_foot;
                delta_foot_world = delta_foot;

                yaw_foot_ = std::atan2(delta_foot_world(1),delta_foot_world(0));

                double r = std::sqrt(delta_foot_world(0)*delta_foot_world(0) + delta_foot_world(1)*delta_foot_world(1));

                gait_generator_->setTrajectoryAmplitude(contact_links_[i], 0, r);
                gait_generator_->setTrajectoryAmplitude(contact_links_[i], 1, yaw_foot_);
                gait_generator_->setTrajectoryAmplitude(contact_links_[i], 2, trj_z_amp_);
            }

        }
        else
        {
            gait_generator_->setTrajectoriesAmplitudes(0.0,
                                                       0.0,
                                                       trj_z_amp_);
        }
*/





    }

    /** @brief Ros subscriber for joypad */
    ros::Subscriber     joy_sub_;
    std::atomic<double> joy_foot_forward_scale_;
    std::atomic<double> joy_foot_lateral_scale_;
    std::atomic<double> joy_base_yaw_scale_;
    std::atomic<double> joy_base_pitch_scale_;
    std::atomic<double> feet_rotation_;
    std::atomic<bool>   joy_start_button_;



    double yaw_dot_ = 0.0; //rad/sec
    double yaw_ = 0.0;
    double yaw_foot_ = 0.0;


};


#endif
