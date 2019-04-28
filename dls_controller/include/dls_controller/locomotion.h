#ifndef LOCOMOTION_H
#define LOCOMOTION_H

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

            ROS_DEBUG("INIT");

            if(trigger_swing)
                state_ = states::SWING;
            break;

        case states::SWING:

            ROS_DEBUG("SWING");

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

            ROS_DEBUG("STANCE");

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
        if(std::strcmp(gait_type.c_str(),"half_trot")==0)
        {
            ROS_INFO("Selected half_trot gait");

            schedule_.push_back(foot_priority_t("lf_foot",0));
            schedule_.push_back(foot_priority_t("rh_foot",0));
            next_feet_to_move_.resize(2);
            max_priority_ = 0;
        }
        else if(std::strcmp(gait_type.c_str(),"trot")==0)
        {
            ROS_INFO("Selected trot gait");

            schedule_.push_back(foot_priority_t("lf_foot",0));
            schedule_.push_back(foot_priority_t("rh_foot",0));

            schedule_.push_back(foot_priority_t("rf_foot",1));
            schedule_.push_back(foot_priority_t("lh_foot",1));
            next_feet_to_move_.resize(2);
            max_priority_ = 1;
        }
        else if(std::strcmp(gait_type.c_str(),"crawl")==0)
        {
            ROS_INFO("Selected crawl gait");

            schedule_.push_back(foot_priority_t("lf_foot",0));
            schedule_.push_back(foot_priority_t("rh_foot",1));
            schedule_.push_back(foot_priority_t("rf_foot",2));
            schedule_.push_back(foot_priority_t("lh_foot",3));
            next_feet_to_move_.resize(1);
            max_priority_ = 3;
        }
        else
        {
            throw std::runtime_error("Wrong Gait!");
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
        time_ = 0.0;
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
    }

protected:

    Eigen::Affine3d reference_;
    Eigen::Affine3d initial_pose_;
    double time_;

};

class SwingOnPlace : public TrajectoryInterface
{

public:

    SwingOnPlace()
    {
        // FIXME
        amp_ = 0.1;
        swing_frequency_ = 1.5;
    }

    void update(const double& period)
    {
        reference_ = initial_pose_;
        reference_.translation().z() +=
                 amp_/2.0 * (0.7 - std::cos(2.0 * M_PI * (swing_frequency_ * time_)));
        time_ += period;
    }


private:

    double amp_;
    double swing_frequency_;

};

class GaitGenerator
{
public:
    GaitGenerator(const double& duty_cycle, const std::vector<std::string>& feet_names, const std::string& gait_type)
    {
        assert(feet_names.size==4);// We assume we are working with a dog
        for(unsigned int i = 0; i<feet_names.size(); i++)
        {
            feet_[feet_names[i]].scheduler = FootScheduler(duty_cycle);
            feet_[feet_names[i]].trajectory.reset(new SwingOnPlace()); // FIXME
            feet_[feet_names[i]].is_in_contact = true;
        }

        gait_buffer_.resize(2);

        for(unsigned int i=0; i<gait_buffer_.size(); i++)
            gait_buffer_[i].reset(new Gait(feet_names,gait_type));

        current_gait_idx_ = 0;
        next_gait_idx_ = 1;
        selected_feet_ = gait_buffer_[current_gait_idx_]->getNextSchedule();

        change_gait_ = false;

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

    bool isInInit(const std::string& foot_name)
    {
        return feet_[foot_name].scheduler.isInit();
    }

    bool isStateChanged(const std::string& foot_name)
    {
        return feet_[foot_name].scheduler.isStateChanged();
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

    typedef std::map<std::string,feet_status_t> feet_t;
    typedef std::shared_ptr<Gait> gait_ptr_t;

    std::vector<std::string> selected_feet_;

    feet_t feet_;
    std::vector<gait_ptr_t > gait_buffer_;
    std::atomic<unsigned int> current_gait_idx_; // atom
    std::atomic<unsigned int> next_gait_idx_; // atom
    std::atomic<bool> change_gait_;

};


} // namespace


#endif
