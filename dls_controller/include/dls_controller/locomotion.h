#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>

namespace dls_controller
{

class FootScheduler
{
public:

    FootScheduler(const double& duty_cycle)
    {
        assert(duty_cycle > 0 && duty_cycle <=1);
        dc_ = duty_cycle;
        reset();
        cnt_ = 0;
        state_ = states::INIT;
    }

    FootScheduler()
        :FootScheduler(0.5)
    {
    }

    bool isSwing()
    {
        if(state_ == states::SWING)
            return true;
        else
            return false;
    }

    bool isInit()
    {
        if(state_ == states::INIT)
            return true;
        else
            return false;
    }

    bool isStance()
    {
        if(state_ == states::STANCE)
            return true;
        else
            return false;
    }

    bool isStateChanged()
    {
        if(prev_state_ != state_)
            return true;
        else
            return false;
    }

    bool isTouchDown()
    {
        if(prev_state_ == states::SWING && state_ == states::STANCE)
            return true;
        else
            return false;
    }

    bool isLiftOff()
    {
        if(prev_state_ == states::INIT && state_ == states::SWING)
            return true;
        else
            return false;
    }

    void setDutyCycle(double duty_cycle)
    {
        assert(duty_cycle > 0 && duty_cycle <=1);
        dc_ = duty_cycle;
    }

    void update(const double& period, const bool& contact, const bool& trigger_swing)
    {

        prev_state_ = state_;

        switch (state_)
        {
        case states::INIT:

            //ROS_DEBUG("INIT");

            if(trigger_swing)
                state_ = states::SWING;
            break;

        case states::SWING:

            //ROS_DEBUG("SWING");

            active_time_ += period;

            if(contact && cnt_++ > 100) // Use a cnt to keep swinging regardless of the contact info
            {
                calculate_times();
                state_ = states::STANCE;
            }
            else
                state_ = states::SWING;

            break;

        case states::STANCE:

            //ROS_DEBUG("STANCE");

            wait_time_-=period;

            if(wait_time_ <= 0.0)
            {
                reset();
                state_ = states::INIT;
            }
            else
                state_ = states::STANCE;

            break;

        default:
            break;

        };
    }

private:
    double dc_;
    double active_time_;
    double total_time_;
    double wait_time_;
    unsigned int cnt_;

    enum states {INIT=0,SWING,STANCE};
    unsigned int state_;
    unsigned int prev_state_;

    void reset()
    {
        wait_time_ = 0.0;
        active_time_ = 0.0;
        total_time_ = 0.0;
        cnt_ = 0;
    }

    void calculate_times()
    {
        total_time_ = active_time_/dc_;
        wait_time_ = total_time_ - active_time_;
    }

};


class Gait
{

public:
    Gait(const std::vector<std::string>& feet_names, const std::string& gait_type)
    {
        //ROS_INFO_STREAM("Selected " << gait_type << " gait");

        if(std::strcmp(gait_type.c_str(),"half_trot")==0)
        {
            schedule_.push_back(foot_priority_t("lf_foot",0));
            schedule_.push_back(foot_priority_t("rh_foot",0));
            next_feet_to_move_.resize(2);
            max_priority_ = 0;
        }
        else if(std::strcmp(gait_type.c_str(),"trot")==0)
        {
            schedule_.push_back(foot_priority_t("lf_foot",0));
            schedule_.push_back(foot_priority_t("rh_foot",0));

            schedule_.push_back(foot_priority_t("rf_foot",1));
            schedule_.push_back(foot_priority_t("lh_foot",1));
            next_feet_to_move_.resize(2);
            max_priority_ = 1;
        }
        else if(std::strcmp(gait_type.c_str(),"crawl")==0)
        {
            schedule_.push_back(foot_priority_t("lf_foot",0));
            schedule_.push_back(foot_priority_t("rh_foot",1));
            schedule_.push_back(foot_priority_t("rf_foot",2));
            schedule_.push_back(foot_priority_t("lh_foot",3));
            next_feet_to_move_.resize(1);
            max_priority_ = 3;
        }
        else
        {
            throw std::runtime_error("Wrong gait type!");
        }

        current_priority_ = 0;
    }

    const std::vector<std::string>& getNextSchedule()
    {

        unsigned int idx = 0;
        for(unsigned int i=0;i < schedule_.size(); i++)
            if(schedule_[i].second == current_priority_)
                next_feet_to_move_[idx++] = schedule_[i].first;

        current_priority_++;
        current_priority_ %= max_priority_+1;

        return next_feet_to_move_;
    }

private:

    typedef std::pair<std::string,unsigned int> foot_priority_t;
    foot_priority_t foot_priority_;
    std::vector<foot_priority_t> schedule_;

    unsigned int current_priority_;
    unsigned int max_priority_;

    std::vector<std::string> next_feet_to_move_;


};

class TrajectoryInterface
{

public:

    TrajectoryInterface()
    {
        reference_ = Eigen::Affine3d::Identity();
        initial_pose_ = Eigen::Affine3d::Identity();
        swing_frequency_ = 3.0;
        time_ = 0.0;
        x_amp_ = 0.0;
        y_amp_ = 0.0;
        z_amp_ = 0.0;

        //trajectory_ended_ = false;
    }

    virtual void update(const double& period) = 0;

    const Eigen::Affine3d& getReference()
    {
        return reference_;
    }

    const Eigen::Affine3d& getInitialPose()
    {
        return initial_pose_;
    }

    void setReferenceToInitialPose()
    {
        reference_ = initial_pose_;
    }

    void setInitialPose(const Eigen::Affine3d& initial_pose)
    {
        initial_pose_ = initial_pose;
    }

    void reset()
    {
        time_ = 0.0;
        //trajectory_ended_ = true;
    }

    void setAmpX(const double& x_amp)
    {
        x_amp_ = x_amp;
    }

    void setAmpY(const double& y_amp)
    {
        y_amp_ = y_amp;
    }

    void setAmpZ(const double& z_amp)
    {
        z_amp_ = z_amp;
    }

    double getAmpX()
    {
        return x_amp_;
    }

    double getAmpY()
    {
        return y_amp_;
    }

    double getAmpZ()
    {
        return z_amp_;
    }

    void setSwingFrequency(const double& swing_frequency)
    {
        if(swing_frequency >= 0.0)
            swing_frequency_ = swing_frequency;
        else {
            ROS_WARN("Swing frequency has to be positive definite!");
        }
    }

protected:

    Eigen::Affine3d reference_;
    Eigen::Affine3d initial_pose_;
    double time_;
    std::atomic<double> swing_frequency_;
    std::atomic<double> x_amp_;
    std::atomic<double> y_amp_;
    std::atomic<double> z_amp_;

    //std::atomic<bool> trajectory_ended_;

};

class Ellipse : public TrajectoryInterface
{

public:

    Ellipse()
    {
        // FIXME
        xyz = Eigen::Vector3d::Zero();
    }

    void update(const double& period)
    {
        //if(trajectory_ended_)
        //    trajectory_ended_=!trajectory_ended_;

        xyz(0) = x_amp_/2 * (1 - std::cos(2* M_PI * (swing_frequency_ * time_)));
        xyz(1) = 0.0;
        xyz(2) = z_amp_ * std::sin(2* M_PI * (swing_frequency_ * time_));

        double c = std::cos(y_amp_);
        double s = std::sin(y_amp_);

        xyz(0) = c * xyz(0) - s * xyz(1);
        xyz(1) = s * xyz(0) + c * xyz(1);
        xyz(2) = xyz(2);

        reference_.pretranslate(xyz);

        if(swing_frequency_*time_<1.0)
            time_ += period;
    }

private:
    Eigen::Vector3d xyz;

};

class GaitGenerator
{
public:
    GaitGenerator(const double& duty_cycle, const std::vector<std::string>& feet_names, const std::string& gait_type, const std::string& trajectory_type)
    {
        assert(feet_names.size()==4);// We assume we are working with a dog
        for(unsigned int i = 0; i<feet_names.size(); i++)
        {
            feet_[feet_names[i]].scheduler = FootScheduler(duty_cycle);
            feet_[feet_names[i]].trajectory.reset(selectTrajectoryType(trajectory_type));
            feet_[feet_names[i]].is_in_contact = true;
        }

        gait_buffer_.resize(2);

        for(unsigned int i=0; i<gait_buffer_.size(); i++)
            gait_buffer_[i].reset(new Gait(feet_names,gait_type));

        current_gait_idx_ = 0;
        next_gait_idx_ = 1;
        selected_feet_ = gait_buffer_[current_gait_idx_]->getNextSchedule();

        change_gait_ = false;
        schedule_changed_ = true; // At the beginning is already changed no?

    }

    void setGaitType(const std::string& gait_type)
    {
        std::vector<std::string> feet_names; // FIXME
        gait_buffer_[next_gait_idx_].reset(new Gait(feet_names,gait_type));
        change_gait_ = true;
    }

    const Eigen::Affine3d& getReference(const std::string& foot_name)
    {
        return feet_[foot_name].trajectory->getReference();
    }

    bool isSwinging(const std::string& foot_name)
    {
        return feet_[foot_name].scheduler.isSwing();
    }

    bool isInStance(const std::string& foot_name)
    {
        return feet_[foot_name].scheduler.isStance();
    }

    bool isInStanceOrInit(const std::string& foot_name)
    {
        return (feet_[foot_name].scheduler.isStance() || feet_[foot_name].scheduler.isInit());
    }

    bool isInInit(const std::string& foot_name)
    {
        return feet_[foot_name].scheduler.isInit();
    }

    bool isStateChanged(const std::string& foot_name)
    {
        return feet_[foot_name].scheduler.isStateChanged();
    }

    bool isTouchDown(const std::string& foot_name)
    {
        return feet_[foot_name].scheduler.isTouchDown();
    }

    bool isLiftOff(const std::string& foot_name)
    {
        return feet_[foot_name].scheduler.isLiftOff();
    }

    bool isScheduleChanged()
    {
        return schedule_changed_;
    }

    bool isAnyFootInLiftOff()
    {
        bool result = false;
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
            result = result || it->second.scheduler.isLiftOff();
        return result;
    }

    bool isAnyFootInTouchDown()
    {
        bool result = false;
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
            result = result || it->second.scheduler.isTouchDown();
        return result;
    }

    void setContact(const std::string& foot_name, const bool& contact)
    {
        feet_[foot_name].is_in_contact = contact;
    }

    void setInitialPose(const std::string& foot_name, const Eigen::Affine3d& initial_pose)
    {
        feet_[foot_name].trajectory->setInitialPose(initial_pose);
    }

    void setDutyCycle(const double& duty_cycle)
    {
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
            it->second.scheduler.setDutyCycle(duty_cycle);
    }

    void setDutyCycle(const std::string& foot_name, const double& duty_cycle)
    {
        feet_[foot_name].scheduler.setDutyCycle(duty_cycle);
    }

    void setSwingFrequency(const std::string& foot_name, const double& swing_frequency)
    {
        feet_[foot_name].trajectory->setSwingFrequency(swing_frequency);
    }

    void setTrajectoryAmplitude(const std::string& foot_name, const unsigned int& id_xyz, const double& amp)
    {
        switch(id_xyz)
        {
        case 0:
            feet_[foot_name].trajectory->setAmpX(amp);
            break;
        case 1:
            feet_[foot_name].trajectory->setAmpY(amp);
            break;
        case 2:
            feet_[foot_name].trajectory->setAmpZ(amp);
            break;
        default:
            ROS_WARN("setTrajectoryAmplitude: Wrong id, possible values are X=0,Y=1,Z=2");
            break;
        };
    }

    void update(const double& period)
    {

        //gait_ptr_t& gait_ = gait_buffer_[current_gait_idx_];

        bool start_swing = true;

        // Set the initial value to each foot (both the scheduled feet and not)
        for(feet_t::iterator it = feet_.begin(); it != feet_.end(); it++)
            it->second.trajectory->setReferenceToInitialPose();

        for(unsigned int i=0; i<selected_feet_.size(); i++)
            if(!feet_[selected_feet_[i]].scheduler.isInit())
            {
                start_swing = false;
                break;
            }

        for(unsigned int i=0; i<selected_feet_.size(); i++)
        {

            feet_[selected_feet_[i]].scheduler.update(period,feet_[selected_feet_[i]].is_in_contact,start_swing);

            if(feet_[selected_feet_[i]].scheduler.isSwing())
            {
                feet_[selected_feet_[i]].trajectory->update(period);
            }
            else
            {
                feet_[selected_feet_[i]].trajectory->reset();
            }
        }

        unsigned int cnt=0;
        for(unsigned int i=0; i<selected_feet_.size(); i++)
            if(feet_[selected_feet_[i]].scheduler.isInit())
                cnt++;
        if(cnt == selected_feet_.size())
        {
            if(change_gait_)
                changeGait();
            selected_feet_ = gait_buffer_[current_gait_idx_]->getNextSchedule();
            schedule_changed_ = true;
        }
        else
        {
            schedule_changed_ = false;
        }

    }

private:

    void changeGait()
    {
        current_gait_idx_ = current_gait_idx_ + 1;
        current_gait_idx_ = current_gait_idx_ % 2;
        next_gait_idx_    = next_gait_idx_    + 1;
        next_gait_idx_    = next_gait_idx_    % 2;

        change_gait_ = false;
    }

    struct feet_status_t
    {
        FootScheduler scheduler;
        std::shared_ptr<TrajectoryInterface> trajectory;
        bool is_in_contact;
    };

    TrajectoryInterface* selectTrajectoryType(const std::string& trajectory_type)
    {
        //ROS_INFO_STREAM("Selected " << trajectory_type << " trajectory");
        if(std::strcmp(trajectory_type.c_str(),"ellipse")==0)
        {
            return new Ellipse();
        }
        else
        {
            throw std::runtime_error("Wrong trajectory type!");
        }
        return NULL;
    }

    typedef std::map<std::string,feet_status_t> feet_t;
    typedef std::shared_ptr<Gait> gait_ptr_t;

    std::vector<std::string> selected_feet_;

    feet_t feet_;
    std::vector<gait_ptr_t > gait_buffer_;
    std::atomic<unsigned int> current_gait_idx_;
    std::atomic<unsigned int> next_gait_idx_;
    std::atomic<bool> change_gait_;
    std::atomic<bool> schedule_changed_;

};


} // namespace


#endif
