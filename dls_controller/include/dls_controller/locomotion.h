#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <atomic>

namespace dls_controller
{

class FootStateMachine
{
public:

    FootStateMachine(const double& duty_cycle)
    {
        assert(duty_cycle > 0 && duty_cycle <=1);
        dc_ = duty_cycle;
        reset();
        cnt_ = 0;
        state_ = states::INIT;
    }

    FootStateMachine()
        :FootStateMachine(0.5)
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

    void triggerSwing()
    {
        trigger_swing_ = true;
    }

    void setDutyCycle(double duty_cycle)
    {
        assert(duty_cycle > 0 && duty_cycle <=1);
        dc_ = duty_cycle;
    }

    void update(const double& period, const bool& contact)
    {

        prev_state_ = state_;

        switch (state_)
        {
        case states::INIT:

            //ROS_DEBUG("INIT");

            if(trigger_swing_)
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
    bool trigger_swing_;

    enum states {INIT=0,SWING,STANCE};
    unsigned int state_;
    unsigned int prev_state_;

    void reset()
    {
        wait_time_ = 0.0;
        active_time_ = 0.0;
        total_time_ = 0.0;
        cnt_ = 0;
        trigger_swing_ = false;
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
        reference_ = initial_pose_ = state_ = Eigen::Affine3d::Identity();
        swing_frequency_ = 3.0;
        time_ = 0.0;
        x_amp_ = 0.0;
        y_amp_ = 0.0;
        z_amp_ = 0.0;

        //trajectory_ended_ = false;
    }

    virtual void update(const double& period) = 0;

    void preview(std::vector<Eigen::Affine3d>& poses)
    {
        assert(swing_frequency_ > 0.0);
        assert(poses.size() > 0);

        double total_time = 1.0/swing_frequency_;
        double period = total_time/poses.size();
        double time = 0.0;
        for(unsigned int i=0; i<poses.size(); i++)
        {
            poses[i] = trajectoryFunction(time);
            time += period;
        }
    }

    const Eigen::Affine3d& getReference()
    {
        return reference_;
    }

    const Eigen::Affine3d& getInitialPose()
    {
        return initial_pose_;
    }

    void setInitialPose(const Eigen::Affine3d& initial_pose)
    {
        initial_pose_ = initial_pose;
    }

    void start()
    {
        time_ = 0.0;
    }

    void stop()
    {
        initial_pose_ = reference_;
    }

    void standBy()
    {
        reference_ = initial_pose_;
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
        else
            ROS_WARN("Swing frequency has to be positive definite!");
    }

protected:

    Eigen::Affine3d reference_;
    Eigen::Affine3d initial_pose_;
    Eigen::Affine3d state_;
    double time_;
    std::atomic<double> swing_frequency_;
    std::atomic<double> x_amp_;
    std::atomic<double> y_amp_;
    std::atomic<double> z_amp_;

    virtual const Eigen::Affine3d& trajectoryFunction(const double& time) = 0;
};

class Ellipse : public TrajectoryInterface
{

public:

    Ellipse()
    {
        xyz = Eigen::Vector3d::Zero();
    }

    void update(const double& period)
    {
        reference_ = trajectoryFunction(time_);

        if(swing_frequency_*time_<1.0)
            time_ += period;
    }

protected:

    const Eigen::Affine3d& trajectoryFunction(const double& time)
    {
        double theta = y_amp_;

        xyz(0) = x_amp_/2 * (1 - std::cos(M_PI * (swing_frequency_ * time)));
        xyz(1) = 0.0;
        xyz(2) = z_amp_ * std::sin(M_PI * (swing_frequency_ * time));

        double c = std::cos(theta);
        double s = std::sin(theta);

        xyz(0) = c * xyz(0) - s * xyz(1);
        xyz(1) = s * xyz(0) + c * xyz(1);
        xyz(2) = xyz(2);

        state_.translation().x() = initial_pose_.translation().x() + xyz(0);
        state_.translation().y() = initial_pose_.translation().y() + xyz(1);
        state_.translation().z() = initial_pose_.translation().z() + xyz(2);

        return state_;
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
            feet_[feet_names[i]].state_machine = FootStateMachine(duty_cycle);
            feet_[feet_names[i]].trajectory.reset(selectTrajectoryType(trajectory_type));
            feet_[feet_names[i]].is_in_contact = true;
        }

        gait_buffer_.resize(2);

        for(unsigned int i=0; i<gait_buffer_.size(); i++)
            gait_buffer_[i].reset(new Gait(feet_names,gait_type));

        current_gait_idx_ = 0;
        next_gait_idx_ = 1;
        scheduled_feet_ = gait_buffer_[current_gait_idx_]->getNextSchedule();

        change_gait_ = false;
        schedule_changed_ = true; // At the beginning is already changed no?

    }

    void setGaitType(const std::string& gait_type)
    {
        std::vector<std::string> feet_names; // FIXME NO-RT
        gait_buffer_[next_gait_idx_].reset(new Gait(feet_names,gait_type));
        change_gait_ = true;
    }

    void getTrajectoryPreview(const std::string& foot_name, std::vector<Eigen::Affine3d>& poses)
    {
        feet_[foot_name].trajectory->preview(poses);
    }

    const Eigen::Affine3d& getReference(const std::string& foot_name)
    {
        return feet_[foot_name].trajectory->getReference();
    }

    bool isSwinging(const std::string& foot_name)
    {
        return feet_[foot_name].state_machine.isSwing();
    }

    bool isInStance(const std::string& foot_name)
    {
        return feet_[foot_name].state_machine.isStance();
    }

    bool isInStanceOrInit(const std::string& foot_name)
    {
        return (feet_[foot_name].state_machine.isStance() || feet_[foot_name].state_machine.isInit());
    }

    bool isInInit(const std::string& foot_name)
    {
        return feet_[foot_name].state_machine.isInit();
    }

    bool isStateChanged(const std::string& foot_name)
    {
        return feet_[foot_name].state_machine.isStateChanged();
    }

    bool isTouchDown(const std::string& foot_name)
    {
        return feet_[foot_name].state_machine.isTouchDown();
    }

    bool isLiftOff(const std::string& foot_name)
    {
        return feet_[foot_name].state_machine.isLiftOff();
    }

    bool isScheduleChanged()
    {
        return schedule_changed_;
    }

    bool isAnyFootInLiftOff()
    {
        bool result = false;
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
            result = result || it->second.state_machine.isLiftOff();
        return result;
    }

    bool isAnyFootInTouchDown()
    {
        bool result = false;
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
            result = result || it->second.state_machine.isTouchDown();
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
            it->second.state_machine.setDutyCycle(duty_cycle);
    }

    void setDutyCycle(const std::string& foot_name, const double& duty_cycle)
    {
        feet_[foot_name].state_machine.setDutyCycle(duty_cycle);
    }

    void setSwingFrequency(const std::string& foot_name, const double& swing_frequency)
    {
        feet_[foot_name].trajectory->setSwingFrequency(swing_frequency);
    }

    void setTrajectoriesAmplitudes(const double& x_amp, const double& y_amp, const double& z_amp)
    {
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
        {
            it->second.trajectory->setAmpX(x_amp);
            it->second.trajectory->setAmpY(y_amp);
            it->second.trajectory->setAmpZ(z_amp);
        }
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

    double getTrajectoryAmplitude(const std::string& foot_name, const unsigned int& id_xyz)
    {
        double amp = 0.0;
        switch(id_xyz)
        {
        case 0:
            amp = feet_[foot_name].trajectory->getAmpX();
            break;
        case 1:
            amp = feet_[foot_name].trajectory->getAmpY();
            break;
        case 2:
            amp = feet_[foot_name].trajectory->getAmpZ();
            break;
        default:
            ROS_WARN("setTrajectoryAmplitude: Wrong id, possible values are X=0,Y=1,Z=2");
            break;
        };
        return amp;
    }

    void update(const double& period)
    {
        // 1) Check if the scheduled feet are all in Init and start the swing if this is the case.
        bool start_swing = true;
        for(unsigned int i=0; i<scheduled_feet_.size(); i++)
            if(!feet_[scheduled_feet_[i]].state_machine.isInit())
            {
                start_swing = false;
                break;
            }
        if(start_swing)
            for(unsigned int i=0; i<scheduled_feet_.size(); i++)
                feet_[scheduled_feet_[i]].state_machine.triggerSwing();

        // 2) Update the trajectories for each foot depending on the state machine status
        for(feet_t::iterator it = feet_.begin(); it != feet_.end(); it++)
        {
            it->second.state_machine.update(period,it->second.is_in_contact);

            if (it->second.state_machine.isSwing())
            {
                if (it->second.state_machine.isLiftOff())
                {
                    it->second.trajectory->start();
                }
                it->second.trajectory->update(period);

                ROS_DEBUG_STREAM("Update trajectory for foot "<< it->first);
            }
            else
            {
                if (it->second.state_machine.isTouchDown())
                {
                    it->second.trajectory->stop();
                }
                it->second.trajectory->standBy();

                ROS_DEBUG_STREAM("StandBy trajectory for foot "<< it->first);
            }
        }

        // 3) If the scheduled feet are all in Init, change the schedule to the next one (i.e. move to the next feet)
        unsigned int cnt = 0;
        for(unsigned int i=0; i<scheduled_feet_.size(); i++)
            if(feet_[scheduled_feet_[i]].state_machine.isInit())
                cnt++;
        if(cnt == scheduled_feet_.size())
        {
            if(change_gait_)
                changeGait();
            scheduled_feet_ = gait_buffer_[current_gait_idx_]->getNextSchedule();
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
        FootStateMachine state_machine;
        std::shared_ptr<TrajectoryInterface> trajectory;
        bool is_in_contact;
    };

    TrajectoryInterface* selectTrajectoryType(const std::string& trajectory_type)
    {
        ROS_INFO_STREAM("Selected " << trajectory_type << " trajectory");
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

    std::vector<std::string> scheduled_feet_;

    feet_t feet_;
    std::vector<gait_ptr_t > gait_buffer_;
    std::atomic<unsigned int> current_gait_idx_;
    std::atomic<unsigned int> next_gait_idx_;
    std::atomic<bool> change_gait_;
    std::atomic<bool> schedule_changed_;

};


} // namespace


#endif
