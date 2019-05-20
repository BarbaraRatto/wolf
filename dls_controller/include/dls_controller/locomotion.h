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

            if(contact && cnt_++ > 10) // Use a cnt to keep swinging regardless of the contact info
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

        if(std::strcmp(gait_type.c_str(),"trot")==0)
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
        else if(std::strcmp(gait_type.c_str(),"bound")==0)
        {
            schedule_.push_back(foot_priority_t("lf_foot",0));
            schedule_.push_back(foot_priority_t("rh_foot",1));
            schedule_.push_back(foot_priority_t("rf_foot",0));
            schedule_.push_back(foot_priority_t("lh_foot",1));
            next_feet_to_move_.resize(2);
            max_priority_ = 1;
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
        reference_ = initial_pose_ = state_ = T_ = Eigen::Affine3d::Identity();
        swing_frequency_ = 5.0;
        time_ = 0.0;
        length_ = 0.0;
        rotation_ = 0.0;
        height_ = 0.0;

        trajectory_finished_ = false;
    }

    virtual void update(const double& period) = 0;

    void preview(std::vector<Eigen::Affine3d>& poses)
    {
        assert(swing_frequency_ > 0.0);
        assert(poses.size() > 0);

        double total_time = 1.0/(swing_frequency_);
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

    bool isFinished()
    {
        return trajectory_finished_;
    }

    void start()
    {
        time_ = 0.0;
        trajectory_finished_ = false;
    }

    void stop()
    {
        initial_pose_ = reference_;
    }

    void standBy()
    {
        reference_ = initial_pose_;
    }

    void setStepLength(const double& length)
    {
        length_ = length;
    }

    void setStepRotation(const double& rotation)
    {
        rotation_ = rotation;
    }

    void setStepHeight(const double& height)
    {
        height_ = height;
    }

    double getStepLength()
    {
        return length_;
    }

    double getStepRotation()
    {
        return rotation_;
    }

    double getStepHeight()
    {
        return height_;
    }

    void setSwingFrequency(const double& swing_frequency)
    {
        if(swing_frequency >= 0.0)
            swing_frequency_ = swing_frequency;
        else
            ROS_WARN("Swing frequency has to be positive definite!");
    }

    double getSwingFrequency()
    {
        return swing_frequency_;
    }

    void setTrajectoryTransformation(const Eigen::Affine3d& T)
    {
        T_ = T;
    }

protected:

    Eigen::Affine3d reference_;
    Eigen::Affine3d initial_pose_;
    Eigen::Affine3d state_;
    Eigen::Affine3d T_;
    double time_;
    std::atomic<double> swing_frequency_;
    std::atomic<double> length_;
    std::atomic<double> rotation_;
    std::atomic<double> height_;
    std::atomic<bool> trajectory_finished_;

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
        else
            trajectory_finished_ = true;
    }

protected:

    const Eigen::Affine3d& trajectoryFunction(const double& time)
    {
        double psi = rotation_;

        xyz(0) = length_/2 * (1 - std::cos(M_PI * (swing_frequency_ * time)));
        xyz(1) = 0.0;
        xyz(2) = height_ * std::sin(M_PI * (swing_frequency_ * time));

        xyz = T_ * xyz;

        double c = std::cos(psi);
        double s = std::sin(psi);

        xyz_rotated(0) = c * xyz(0) - s * xyz(1);
        xyz_rotated(1) = s * xyz(0) + c * xyz(1);
        xyz_rotated(2) = xyz(2);

        state_.translation().x() = initial_pose_.translation().x() + xyz_rotated(0);
        state_.translation().y() = initial_pose_.translation().y() + xyz_rotated(1);
        state_.translation().z() = initial_pose_.translation().z() + xyz_rotated(2);

        return state_;
    }

private:
    Eigen::Vector3d xyz;
    Eigen::Vector3d xyz_rotated;

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
        activate_swing_ = false;

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

    void setTrajectoryTransformation(const Eigen::Affine3d& T)
    {
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
            it->second.trajectory->setTrajectoryTransformation(T);
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

    double getSwingFrequency(const std::string& foot_name)
    {
        return feet_[foot_name].trajectory->getSwingFrequency();
    }

    void setTrajectoryAmplitude(const double& length, const double& rotation, const double& height)
    {
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
        {
            it->second.trajectory->setStepLength(length);
            it->second.trajectory->setStepRotation(rotation);
            it->second.trajectory->setStepHeight(height);
        }
    }

    void setTrajectoryAmplitude(const std::string& foot_name, const unsigned int& id, const double& amp)
    {
        switch(id)
        {
        case 0:
            feet_[foot_name].trajectory->setStepLength(amp);
            break;
        case 1:
            feet_[foot_name].trajectory->setStepRotation(amp);
            break;
        case 2:
            feet_[foot_name].trajectory->setStepHeight(amp);
            break;
        default:
            ROS_WARN("setTrajectoryAmplitude: Wrong id, possible values are X=0,Y=1,Z=2");
            break;
        };
    }

    double getTrajectoryAmplitude(const std::string& foot_name, const unsigned int& id)
    {
        double amp = 0.0;
        switch(id)
        {
        case 0:
            amp = feet_[foot_name].trajectory->getStepLength();
            break;
        case 1:
            amp = feet_[foot_name].trajectory->getStepRotation();
            break;
        case 2:
            amp = feet_[foot_name].trajectory->getStepHeight();
            break;
        default:
            ROS_WARN("setTrajectoryAmplitude: Wrong id, possible values are X=0,Y=1,Z=2");
            break;
        };
        return amp;
    }

    void activateSwing()
    {
        activate_swing_ = true;
    }

    void deactivateSwing()
    {
        activate_swing_ = false;
    }

    void update(const double& period)
    {
        // 1) Check if the scheduled feet are all in Init and start the swing if this is the case.
        bool scheduled_feet_are_init = true;
        for(unsigned int i=0; i<scheduled_feet_.size(); i++)
            if(!feet_[scheduled_feet_[i]].state_machine.isInit())
            {
                scheduled_feet_are_init = false;
                break;
            }
        if(scheduled_feet_are_init && activate_swing_)
            for(unsigned int i=0; i<scheduled_feet_.size(); i++)
                feet_[scheduled_feet_[i]].state_machine.triggerSwing();

        // 2) Update the trajectories for each foot depending on the state machine status
        for(feet_t::iterator it = feet_.begin(); it != feet_.end(); it++)
        {
            //it->second.state_machine.update(period,it->second.is_in_contact); //ClosedLoop
            it->second.state_machine.update(period,it->second.trajectory->isFinished()); // OpenLoop

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
    std::atomic<bool> activate_swing_;

};


class RobotCmds
{

public:

    enum cmds {HOLD=0,MOVE_FEET,ROTATE_BASE_YAW};

    RobotCmds(const std::vector<std::string>& feet, const double & default_length = 0.0, const double & default_height = 0.0)
    {
        assert(feet.size() > 0);
        for(unsigned int i=0;i<feet.size();i++)
        {
            steps_length_[feet[i]] = default_length;
            steps_rotation_[feet[i]] = 0.0;
            steps_height_[feet[i]] = default_height;
        }
    }

    void update()
    {




    }

    bool trackingActive() {return tracking_active_;}
    void startTracking()  {tracking_active_ = true;}
    void stopTracking()   {tracking_active_ = false;}
    // Command type
    unsigned int getCmd()   {return cmd_;}
    void setCmd(const unsigned int& cmd)   {cmd_ = cmd;}
    // Base commands
    void setBaseRollAngle(const double& roll)   {base_roll_   = roll;}
    void setBasePitchAngle(const double& pitch) {base_pitch_  = pitch;}
    void setBaseYawAngle(const double& yaw)     {base_yaw_    = yaw;}
    void setBaseHeight(const double& height)    {base_height_ = height;}
    //Feet commands
    void setStepLength(const std::string& foot_name, const double& length) {steps_length_[foot_name]   = length;}
    void setStepRotation(const std::string& foot_name, const double& rot)  {steps_rotation_[foot_name] = rot;}
    void setStepHeight(const std::string& foot_name, const double& height) {steps_height_[foot_name]   = height;}
    void setStepLength(const double& length)
    {
        for(map_t::iterator it = steps_length_.begin(); it!=steps_length_.end(); ++it)
            it->second = length;
    }
    void setStepRotation(const double& rot)
    {
        for(map_t::iterator it = steps_rotation_.begin(); it!=steps_rotation_.end(); ++it)
            it->second = rot;
    }
    void setStepHeight(const double& height)
    {
        for(map_t::iterator it = steps_height_.begin(); it!=steps_height_.end(); ++it)
            it->second = height;
    }
    // Gets
    double getBaseRollAngle()  {return base_roll_  ;}
    double getBasePitchAngle() {return base_pitch_ ;}
    double getBaseYawAngle()   {return base_yaw_   ;}
    double getBaseHeight()     {return base_height_;}
    double getStepLength(const std::string& foot_name)     {return steps_length_[foot_name]   ;}
    double getStepRotation(const std::string& foot_name)   {return steps_rotation_[foot_name] ;}
    double getStepHeight(const std::string& foot_name)     {return steps_height_[foot_name]   ;}

private:

    typedef std::map<std::string,std::atomic<double>> map_t;

    std::atomic<unsigned int> cmd_;
    std::atomic<bool>   tracking_active_;
    std::atomic<double> base_roll_  ;
    std::atomic<double> base_pitch_ ;
    std::atomic<double> base_yaw_   ;
    std::atomic<double> base_height_;
    map_t steps_length_;
    map_t steps_rotation_;
    map_t steps_height_;

};

} // namespace


#endif
