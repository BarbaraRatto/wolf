#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <atomic>
#include <XBotInterface/ModelInterface.h>
#include <wb_controller/geometry.h>
#include <wb_controller/utils.h>

namespace wb_controller
{

class FootStateMachine
{
public:

    FootStateMachine()
    {
        dc_ = 0.8;
        swing_frequency_ = 0.0;
        reset();
        state_ = states::INIT;
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

    void setSwingFrequency(double swing_frequency)
    {
        assert(swing_frequency >= 0);
        swing_frequency_ = swing_frequency;
    }

    double getDutyCycle()
    {
        return dc_;
    }

    void update(const double& period, const bool& contact)
    {

        prev_state_ = state_;

        double half_swing_time = 0.0;

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

            if(swing_frequency_>0)
                half_swing_time = 1/(2*swing_frequency_);


            if(contact && active_time_>=half_swing_time)
            {
                calculateTimes();
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

    double active_time_;
    double total_time_;
    double wait_time_;
    bool trigger_swing_;
    std::atomic<double> swing_frequency_;
    std::atomic<double> dc_;

    enum states {INIT=0,SWING,STANCE};
    unsigned int state_;
    unsigned int prev_state_;

    void reset()
    {
        wait_time_ = 0.0;
        active_time_ = 0.0;
        total_time_ = 0.0;
        trigger_swing_ = false;
    }

    void calculateTimes()
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
        ROS_INFO_STREAM("Selected " << gait_type << " gait");

        assert(feet_names.size() == N_LEGS);

        auto ordered_feet_names = sortByLegName(feet_names);

        if(std::strcmp(gait_type.c_str(),"trot")==0)
        {
            schedule_.push_back(foot_priority_t(ordered_feet_names[leg_id::LF],0));
            schedule_.push_back(foot_priority_t(ordered_feet_names[leg_id::RH],0));
            schedule_.push_back(foot_priority_t(ordered_feet_names[leg_id::RF],1));
            schedule_.push_back(foot_priority_t(ordered_feet_names[leg_id::LH],1));
            next_feet_to_move_.resize(2);
            max_priority_ = 1;
        }
        else if(std::strcmp(gait_type.c_str(),"crawl")==0)
        {
            schedule_.push_back(foot_priority_t(ordered_feet_names[leg_id::LF],0));
            schedule_.push_back(foot_priority_t(ordered_feet_names[leg_id::RH],1));
            schedule_.push_back(foot_priority_t(ordered_feet_names[leg_id::RF],2));
            schedule_.push_back(foot_priority_t(ordered_feet_names[leg_id::LH],3));
            next_feet_to_move_.resize(1);
            max_priority_ = 3;
        }
        else if(std::strcmp(gait_type.c_str(),"bound")==0)
        {
            schedule_.push_back(foot_priority_t(ordered_feet_names[leg_id::LF],0));
            schedule_.push_back(foot_priority_t(ordered_feet_names[leg_id::RH],1));
            schedule_.push_back(foot_priority_t(ordered_feet_names[leg_id::RF],0));
            schedule_.push_back(foot_priority_t(ordered_feet_names[leg_id::LH],1));
            next_feet_to_move_.resize(2);
            max_priority_ = 1;
        }
        else if(std::strcmp(gait_type.c_str(),"one_foot_lf")==0)
        {
            schedule_.push_back(foot_priority_t(ordered_feet_names[leg_id::LF],0));
            next_feet_to_move_.resize(1);
            max_priority_ = 0;
        }
        else if(std::strcmp(gait_type.c_str(),"one_foot_rh")==0)
        {
            schedule_.push_back(foot_priority_t(ordered_feet_names[leg_id::RH],0));
            next_feet_to_move_.resize(1);
            max_priority_ = 0;
        }
        else if(std::strcmp(gait_type.c_str(),"one_foot_rf")==0)
        {
            schedule_.push_back(foot_priority_t(ordered_feet_names[leg_id::RF],0));
            next_feet_to_move_.resize(1);
            max_priority_ = 0;
        }
        else if(std::strcmp(gait_type.c_str(),"one_foot_lh")==0)
        {
            schedule_.push_back(foot_priority_t(ordered_feet_names[leg_id::LH],0));
            next_feet_to_move_.resize(1);
            max_priority_ = 0;
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
        pose_reference_ = initial_pose_ = pose_ = Eigen::Affine3d::Identity();
        twist_reference_.setZero();
        twist_.setZero();
        swing_frequency_ = 0.0;
        time_ = 0.0;
        length_ = 0.0;
        heading_ = 0.0;
        heading_rate_ = 0.0;
        height_ = 0.0;

        trajectory_finished_ = true;
    }

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
        return pose_reference_;
    }

    const Eigen::Vector6d& getReferenceDot()
    {
        return twist_reference_;
    }

    const Eigen::Affine3d& getInitialPose()
    {
        return initial_pose_;
    }

    void setInitialPose(const Eigen::Affine3d& initial_pose)
    {
#ifdef OPEN_LOOP_TRAJECTORY
        // Open loop trajectory
        initial_pose_ = pose_reference_;
#else
        // Closed loop trajectory: to be used if the tracking is good
        initial_pose_ = initial_pose;
        pose_reference_ = initial_pose;
#endif
    }

    bool isFinished()
    {
        return trajectory_finished_;
    }

    void start()
    {
        time_ = 0.0;
        twist_reference_.setZero();
        trajectory_finished_ = false;
    }

    void stop()
    {
        twist_reference_.setZero();
    }

    void standBy()
    {
        twist_reference_.setZero();
    }

    void setStepHeadingRate(const double& heading_rate)
    {
        heading_rate_ = heading_rate;
    }

    void setStepLength(const double& length)
    {
        length_ = length;
    }

    void setStepHeading(const double& heading)
    {
        heading_ = heading;
    }

    void setStepHeight(const double& height)
    {
        height_ = height;
    }

    double getStepLength()
    {
        return length_;
    }

    double getStepHeading()
    {
        return heading_;
    }

    double getStepHeadingRate()
    {
        return heading_rate_;
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

    void update(const double& period)
    {
        pose_reference_  = trajectoryFunction(time_);
        twist_reference_ = trajectoryFunctionDot(time_);

        if(swing_frequency_*time_<1.0)
            time_ += period;
        else
            trajectory_finished_ = true;
    }

protected:

    /** @brief Pose reference */
    Eigen::Affine3d pose_reference_;
    /** @brief Twist reference */
    Eigen::Vector6d twist_reference_;
    /** @brief Initial pose for the trajectory generation */
    Eigen::Affine3d initial_pose_;
    /** @brief Internal pose of the trajectory */
    Eigen::Affine3d pose_;
    /** @brief Internal twist of the trajectory */
    Eigen::Vector6d twist_;
    double time_;
    std::atomic<double> swing_frequency_;
    std::atomic<double> length_;
    std::atomic<double> heading_;
    std::atomic<double> heading_rate_;
    std::atomic<double> height_;
    bool trajectory_finished_;

    virtual const Eigen::Affine3d& trajectoryFunction(const double& time) = 0;
    virtual const Eigen::Vector6d& trajectoryFunctionDot(const double& time) = 0;
};

class Ellipse : public TrajectoryInterface
{

public:

    Ellipse()
    {
        xyz = xyz_rotated = xyz_dot = Eigen::Vector3d::Zero();
        world_Rz_swing = Sz = Ear = Eigen::Matrix3d::Zero();
    }

protected:

    const Eigen::Affine3d& trajectoryFunction(const double& time)
    {
        const double& yaw = heading_;

        xyz(0) = length_/2 * (1 - std::cos(M_PI * (swing_frequency_ * time)));
        xyz(1) = 0.0;
        xyz(2) = height_ * std::sin(M_PI * (swing_frequency_ * time));

        double c = std::cos(yaw);
        double s = std::sin(yaw);

        world_Rz_swing(0,0) = c;
        world_Rz_swing(0,1) = -s;
        world_Rz_swing(1,0) = s;
        world_Rz_swing(1,1) = c;
        world_Rz_swing(2,2) = 1;

        xyz_rotated = world_Rz_swing * xyz;

        pose_.translation() = initial_pose_.translation() + xyz_rotated;

        return pose_;
    }

    const Eigen::Vector6d& trajectoryFunctionDot(const double& time)
    {
        const double& yaw_rate = heading_rate_;
        const double& yaw = heading_;

        rpy_rates.setZero();
        omegas.setZero();
        rpy.setZero();

        rpy_rates(2) = yaw_rate;
        rpy(2) = yaw;

        rpyToEar(rpy,Ear);

        omegas = Ear * rpy_rates;

        Sz(0,1) = -omegas(2);
        Sz(1,0) = -omegas(2);

        xyz_dot(0) = M_PI * swing_frequency_ * length_/2 * std::sin(M_PI * (swing_frequency_ * time));
        xyz_dot(1) = 0.0;
        xyz_dot(2) = M_PI * swing_frequency_ * height_ * std::cos(M_PI * (swing_frequency_ * time));

        twist_.setZero(); // No angular velocities
        twist_.head(3) = Sz * xyz_rotated + world_Rz_swing * xyz_dot;

        return twist_;
    }

private:

    Eigen::Vector3d xyz;
    Eigen::Vector3d xyz_dot;
    Eigen::Vector3d xyz_rotated;
    Eigen::Matrix3d world_Rz_swing;
    Eigen::Matrix3d Sz;
    Eigen::Matrix3d Ear;

    Eigen::Vector3d rpy;
    Eigen::Vector3d rpy_rates;
    Eigen::Vector3d omegas;
};

class GaitGenerator
{
public:

    /**
     * @brief Shared pointer to GaitGenerator
     */
    typedef std::shared_ptr<GaitGenerator> Ptr;

    /**
     * @brief Shared pointer to const GaitGenerator
     */
    typedef std::shared_ptr<const GaitGenerator> ConstPtr;

    GaitGenerator(const std::vector<std::string>& feet_names, const std::vector<std::string>& hips_names, const std::string& gait_type, const std::string& trajectory_type)
    {
        assert(feet_names.size()==N_LEGS);// We assume we are working with a dog
        assert(hips_names.size()==N_LEGS);
        feet_names_ = feet_names;
        hips_names_ = hips_names;
        for(unsigned int i = 0; i<feet_names.size(); i++)
        {
            feet_[feet_names[i]].state_machine.reset(new FootStateMachine());
            feet_[feet_names[i]].trajectory.reset(selectTrajectoryType(trajectory_type));
            feet_[feet_names[i]].is_in_contact = true;
            feet_[feet_names[i]].initial_pose = Eigen::Affine3d::Identity();
        }

        setSwingFrequency(3.3);

        gait_buffer_.resize(2);

        for(unsigned int i=0; i<gait_buffer_.size(); i++)
            gait_buffer_[i].reset(new Gait(feet_names,gait_type));

        current_gait_idx_ = 0;
        next_gait_idx_ = 1;
        scheduled_feet_ = gait_buffer_[current_gait_idx_]->getNextSchedule();

        change_gait_ = false;
        schedule_changed_ = true; // At the beginning is already changed no?
        activate_swing_ = false;
        use_haptic_contact_loop_ = false;

        gait_type_ = gait_type;
    }

    void setGaitType(const std::string& gait_type)
    {
        gait_buffer_[next_gait_idx_].reset(new Gait(feet_names_,gait_type));
        change_gait_ = true;
        gait_type_ = gait_type;
    }

    void getTrajectoryPreview(const std::string& foot_name, std::vector<Eigen::Affine3d>& poses)
    {
        feet_[foot_name].trajectory->preview(poses);
    }

    const Eigen::Affine3d& getReference(const std::string& foot_name)
    {
        return feet_[foot_name].trajectory->getReference();
    }

    const Eigen::Vector6d& getReferenceDot(const std::string& foot_name)
    {
        return feet_[foot_name].trajectory->getReferenceDot();
    }

    bool isSwinging(const std::string& foot_name)
    {
        return feet_[foot_name].state_machine->isSwing();
    }

    bool isInStance(const std::string& foot_name)
    {
        return feet_[foot_name].state_machine->isStance();
    }

    bool isInStanceOrInit(const std::string& foot_name)
    {
        return (feet_[foot_name].state_machine->isStance() || feet_[foot_name].state_machine->isInit());
    }

    bool isInInit(const std::string& foot_name)
    {
        return feet_[foot_name].state_machine->isInit();
    }

    bool isStateChanged(const std::string& foot_name)
    {
        return feet_[foot_name].state_machine->isStateChanged();
    }

    bool isTouchDown(const std::string& foot_name)
    {
        return feet_[foot_name].state_machine->isTouchDown();
    }

    bool isLiftOff(const std::string& foot_name)
    {
        return feet_[foot_name].state_machine->isLiftOff();
    }

    bool isScheduleChanged()
    {
        return schedule_changed_;
    }

    bool isAnyFootInLiftOff()
    {
        bool result = false;
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
            result = result || it->second.state_machine->isLiftOff();
        return result;
    }

    bool isAnyFootInTouchDown()
    {
        bool result = false;
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
            result = result || it->second.state_machine->isTouchDown();
        return result;
    }

    void enableHapticContactLoop()
    {
        use_haptic_contact_loop_ = true;
    }

    void disableHapticContactLoop()
    {
        use_haptic_contact_loop_ = false;
    }

    void setContact(const std::string& foot_name, const bool& contact)
    {
        feet_[foot_name].is_in_contact = contact;
    }

    const bool& getContact(const std::string& foot_name)
    {
        return feet_[foot_name].is_in_contact;
    }

    void setInitialPose(const std::string& foot_name, const Eigen::Affine3d& initial_pose)
    {
        feet_[foot_name].trajectory->setInitialPose(initial_pose);
        //feet_[foot_name].initial_pose = initial_pose;
    }

    double getDutyCycle(const std::string& foot_name)
    {
        return feet_[foot_name].state_machine->getDutyCycle();
    }

    void setDutyCycle(const double& duty_cycle)
    {
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
            it->second.state_machine->setDutyCycle(duty_cycle);
    }

    void setDutyCycle(const std::string& foot_name, const double& duty_cycle)
    {
        feet_[foot_name].state_machine->setDutyCycle(duty_cycle);
    }

    void setSwingFrequency(const double& swing_frequency)
    {
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
        {
            it->second.trajectory->setSwingFrequency(swing_frequency);
            it->second.state_machine->setSwingFrequency(swing_frequency);
        }
    }

    void setSwingFrequency(const std::string& foot_name, const double& swing_frequency)
    {
        feet_[foot_name].trajectory->setSwingFrequency(swing_frequency);
        feet_[foot_name].state_machine->setSwingFrequency(swing_frequency);
    }

    double getSwingFrequency(const std::string& foot_name)
    {
        return feet_[foot_name].trajectory->getSwingFrequency();
    }

    const std::vector<std::string>& getFeetNames()
    {
        return feet_names_;
    }

    const std::vector<std::string>& getHipsNames()
    {
        return hips_names_;
    }

    const std::string& getGaitType()
    {
        return gait_type_;
    }

    void setStepLength(const double& length)
    {
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
            it->second.trajectory->setStepLength(length);
    }

    void setStepHeading(const double& heading)
    {
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
            it->second.trajectory->setStepHeading(heading);
    }

    void setStepHeight(const double& height)
    {
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
            it->second.trajectory->setStepHeight(height);
    }

    void setStepHeadingRate(const double& heading_rate)
    {
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
            it->second.trajectory->setStepHeadingRate(heading_rate);
    }

    void setStepLength(const std::string& foot_name, const double& length)
    {
        feet_[foot_name].trajectory->setStepLength(length);
    }

    void setStepHeading(const std::string& foot_name, const double& heading)
    {
       feet_[foot_name].trajectory->setStepHeading(heading);
    }

    void setStepHeight(const std::string& foot_name, const double& height)
    {
        feet_[foot_name].trajectory->setStepHeight(height);
    }

    void setStepHeadingRate(const std::string& foot_name, const double& heading_rate)
    {
        feet_[foot_name].trajectory->setStepHeadingRate(heading_rate);
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
            if(!feet_[scheduled_feet_[i]].state_machine->isInit())
            {
                scheduled_feet_are_init = false;
                break;
            }
        if(scheduled_feet_are_init && activate_swing_)
            for(unsigned int i=0; i<scheduled_feet_.size(); i++)
                feet_[scheduled_feet_[i]].state_machine->triggerSwing();

        // 2) Update the trajectories for each foot depending on the state machine status
        for(feet_t::iterator it = feet_.begin(); it != feet_.end(); it++)
        {

            //if(!use_haptic_contact_loop_)
            //    it->second.is_in_contact = it->second.trajectory->isFinished(); // OpenLoop

            it->second.state_machine->update(period,it->second.is_in_contact);

            if (it->second.state_machine->isSwing())
            {
                if (it->second.state_machine->isLiftOff())
                {
                    it->second.trajectory->start();
                }
                it->second.trajectory->update(period);

                ROS_DEBUG_STREAM("Update trajectory for foot "<< it->first);
            }
            else
            {
                if (it->second.state_machine->isTouchDown())
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
            if(feet_[scheduled_feet_[i]].state_machine->isInit())
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
        std::shared_ptr<FootStateMachine> state_machine;
        std::shared_ptr<TrajectoryInterface> trajectory;
        bool is_in_contact;
        Eigen::Affine3d initial_pose;
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

    bool use_haptic_contact_loop_;

    std::vector<std::string> feet_names_;
    std::vector<std::string> hips_names_;

    std::string gait_type_;

};

} // namespace


#endif
