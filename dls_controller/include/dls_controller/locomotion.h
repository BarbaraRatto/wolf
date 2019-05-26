#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <atomic>
#include <XBotInterface/ModelInterface.h>

//#define HAPTIC_CLOSED_LOOP


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


#ifdef HAPTIC_CLOSED_LOOP
            if(contact && cnt_++ > 10) // Use a cnt to keep swinging regardless of the contact info
#else
            if(contact)
#endif
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
        heading_ = 0.0;
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

    void stop() // Take the last reference as next initial pose
    {
        initial_pose_ = reference_; // This is useful if the trajectory has to be computed w.r.t world
    }

    void stop(const Eigen::Affine3d& initial_pose_next_swing)
    {
        initial_pose_ = initial_pose_next_swing;
    }

    void standBy()
    {
        reference_ = initial_pose_;
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
    std::atomic<double> heading_;
    std::atomic<double> height_;
    bool trajectory_finished_;

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
        double psi = heading_;

        xyz(0) = length_/2 * (1 - std::cos(M_PI * (swing_frequency_ * time)));
        xyz(1) = 0.0;
#ifdef HAPTIC_CLOSED_LOOP
        xyz(2) = height_ * std::sin(M_PI * (swing_frequency_ * time)) - height_/10.0;
#else
        xyz(2) = height_ * std::sin(M_PI * (swing_frequency_ * time));
#endif

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

    /**
     * @brief Shared pointer to GaitGenerator
     */
    typedef std::shared_ptr<GaitGenerator> Ptr;

    /**
     * @brief Shared pointer to const GaitGenerator
     */
    typedef std::shared_ptr<const GaitGenerator> ConstPtr;

    GaitGenerator(const double& duty_cycle, const std::vector<std::string>& feet_names, const std::string& gait_type, const std::string& trajectory_type)
    {
        assert(feet_names.size()==4);// We assume we are working with a dog
        feet_names_ = feet_names;
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

    const std::vector<std::string>& getFeetNames()
    {
        return feet_names_;
    }

    void setTrajectoryAmplitude(const double& length, const double& heading, const double& height)
    {
        for(feet_t::iterator it = feet_.begin(); it!=feet_.end(); ++it)
        {
            it->second.trajectory->setStepLength(length);
            it->second.trajectory->setStepHeading(heading);
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
            feet_[foot_name].trajectory->setStepHeading(amp);
            break;
        case 2:
            feet_[foot_name].trajectory->setStepHeight(amp);
            break;
        default:
            ROS_WARN("setTrajectoryAmplitude: Wrong id, possible values are Length=0,Heading=1,Height=2");
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
            amp = feet_[foot_name].trajectory->getStepHeading();
            break;
        case 2:
            amp = feet_[foot_name].trajectory->getStepHeight();
            break;
        default:
            ROS_WARN("setTrajectoryAmplitude: Wrong id, possible values Length=0,Heading=1,Height=2");
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
#ifdef HAPTIC_CLOSED_LOOP
            it->second.state_machine.update(period,it->second.is_in_contact); //ClosedLoop with Haptic
#else
            it->second.state_machine.update(period,it->second.trajectory->isFinished()); // OpenLoop
#endif

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

    std::vector<std::string> feet_names_;

};

class CommandsInterface
{

public:

    enum cmd_t {HOLD=0,BASE_VELOCITY_ONLY_LINEAR,BASE_VELOCITY_ONLY_ANGULAR,BASE_VELOCITY_FULL};

    CommandsInterface(GaitGenerator::Ptr gait_generator, XBot::ModelInterface::Ptr xbot_model)
    {
        // FIXME:
        // Add max safety height and length
        assert(gait_generator);
        gait_generator_ = gait_generator;
        assert(xbot_model);
        xbot_model_ = xbot_model;
        const std::vector<std::string>& feet_names = gait_generator_->getFeetNames();
        for(unsigned int i=0;i<feet_names.size();i++)
        {
            steps_length_[feet_names[i]] = 0.0;
            steps_heading_[feet_names[i]] = 0.0;
            steps_height_[feet_names[i]] = 0.0;
        }

        base_linear_velocity_scale_x_ = 0.0;
        base_linear_velocity_scale_y_ = 0.0;
        base_linear_velocity_scale_z_ = 0.0;

        base_angular_velocity_scale_roll_ = 0.0;
        base_angular_velocity_scale_pitch_ = 0.0;
        base_angular_velocity_scale_yaw_ = 0.0;

        base_linear_velocity_max_ = 0.5;
        base_angular_velocity_max_ = 1.0;

        base_rotation_reference_ = Eigen::Matrix3d::Identity();
        base_position_ = base_orientation_ = Eigen::Vector3d::Zero();

        cmd_ = cmd_t::HOLD;

        // FIXME hardcoded names
        hips_.resize(4);
        hips_[0] = "lf_hipassembly";
        hips_[1] = "rf_hipassembly";
        hips_[2] = "lh_hipassembly";
        hips_[3] = "rh_hipassembly";


        base_X_hip_foot_offsets_.resize(4);
        hf_X_base_hip_offsets_.resize(4);
        for(unsigned int i=0; i<base_X_hip_foot_offsets_.size(); i++)
        {
            base_X_hip_foot_offsets_[i].setZero();
            hf_X_base_hip_offsets_[i].setZero();
        }

        offset_applied_ = false;
    }

    void update(const double& period,const Eigen::Vector3d& base_position) // OpenLoop Orientation
    {
        update(period,base_position,base_orientation_);
    }

    void update(const double& period) // OpenLoop
    {
        update(period,base_position_,base_orientation_);
    }

    void update(const double& period, const Eigen::Vector3d& base_position, const Eigen::Vector3d& base_orientation) // ClosedLoop
    {
        unsigned int cmd = cmd_;
        const std::vector<std::string>& feet_names = gait_generator_->getFeetNames();

        xbot_model_->getPose("base_link",world_T_base_);

        world_R_hf_ = Eigen::Matrix3d::Identity();
        yaw_base_ = std::atan2(world_T_base_.linear()(1,0),world_T_base_.linear()(0,0));
        world_R_hf_ = Eigen::AngleAxisd(yaw_base_,Eigen::Vector3d::UnitZ());

        world_R_base_ = world_T_base_.linear();
        hf_R_base_ = world_R_hf_.transpose() * world_R_base_;

        setHipOffset();

        switch(cmd)
        {

        case cmd_t::HOLD:
            break;

        case cmd_t::BASE_VELOCITY_ONLY_LINEAR:
            calculatePosition(period,base_position);
            base_angular_velocity_.fill(0.0);
            break;

        case cmd_t::BASE_VELOCITY_ONLY_ANGULAR:
            calculateOrientation(period,base_orientation);
            base_linear_velocity_.fill(0.0);
            break;

        case cmd_t::BASE_VELOCITY_FULL:
            calculatePosition(period,base_position);
            calculateOrientation(period,base_orientation);
            break;
        };

        const Eigen::Vector3d& hf_base_linear_velocity = base_linear_velocity_;
        const Eigen::Vector3d& hf_base_angular_velocity = base_angular_velocity_;

        for(unsigned int i=0; i<feet_names.size(); i++)
        {            
            if(gait_generator_->isSwinging(feet_names[i]))
            {

                ROS_INFO_STREAM("*********");
                ROS_INFO_STREAM("Swinging foot "<<feet_names[i]);

                xbot_model_->getPose(feet_names[i],world_T_foot_); // Should it be in respect of the hf?
                xbot_model_->getPose(hips_[i],world_T_hip_);
                xbot_model_->getPose(hips_[i],"base_link",base_T_hip_);

                hf_delta_hip_.setZero();
                hf_delta_hip_(0) = hf_base_linear_velocity(0)*1.0/gait_generator_->getSwingFrequency(feet_names[i]);
                hf_delta_hip_(1) = hf_base_linear_velocity(1)*1.0/gait_generator_->getSwingFrequency(feet_names[i]);

                //hf_X_hip_ = hf_R_base_ * base_T_hip_.translation();
                hf_delta_heding_ = Eigen::Vector3d(0,0,hf_base_angular_velocity(2)*1.0/gait_generator_->getSwingFrequency(feet_names[i])).cross(hf_X_base_hip_offsets_[i]);

                ROS_INFO_STREAM("hf_delta_heding_: "<<hf_delta_heding_.transpose());

                hf_delta_hip_(0)+= hf_delta_heding_(0);
                hf_delta_hip_(1)+= hf_delta_heding_(1);

                ROS_INFO_STREAM("hf_delta_hip_: "<<hf_delta_hip_.transpose());

                world_delta_hip_ = world_R_hf_ * hf_delta_hip_;

                ROS_INFO_STREAM("world_delta_hip_: "<<world_delta_hip_.transpose());

                world_X_hip_ = world_R_hf_ * hf_X_base_hip_offsets_[i] + world_T_base_.translation();
                //Eigen::Vector3d world_X_hip_foot = world_T_foot_.translation()- world_T_hip_.translation();

                ROS_INFO_STREAM("world_X_hip_: "<<world_X_hip_.transpose());

                world_X_hip_foot_ = world_T_foot_.translation() - world_X_hip_;

                ROS_INFO_STREAM("world_X_hip_foot_: "<<world_X_hip_foot_.transpose());

                world_X_hip_foot_offset_ = world_R_base_ * base_X_hip_foot_offsets_[i];
                world_X_hip_foot_offset_(2) = 0;

                ROS_INFO_STREAM("world_X_hip_foot_offset_: "<<world_X_hip_foot_offset_.transpose());

                world_delta_foot_.setZero();
                world_delta_foot_.head(2) =  world_X_hip_foot_offset_.head(2) + world_delta_hip_.head(2) - world_X_hip_foot_.head(2);

                ROS_INFO_STREAM("world_delta_foot_: "<<world_delta_foot_.transpose());

                steps_length_[feet_names[i]]   = std::sqrt(world_delta_foot_(0)*world_delta_foot_(0) + world_delta_foot_(1)*world_delta_foot_(1));
                steps_heading_[feet_names[i]] = std::atan2(world_delta_foot_(1),world_delta_foot_(0));
                steps_height_[feet_names[i]]   = 0.05; // FIXME

                //getchar();
            }
            else
            {
                steps_length_[feet_names[i]]   = 0.0;
                steps_heading_[feet_names[i]] = 0.0;
                steps_height_[feet_names[i]]   = 0.0;
            }

            ROS_DEBUG_STREAM("steps_length["<<feet_names[i]<<"]: "<<steps_length_[feet_names[i]]);
            ROS_DEBUG_STREAM("steps_heading_["<<feet_names[i]<<"]: "<<steps_heading_[feet_names[i]]);
            ROS_DEBUG_STREAM("steps_height_["<<feet_names[i]<<"]: "<<steps_height_[feet_names[i]]);

        }
        // FIXME: Look up in stop().
        // For translations I do not need to set the next step starting from the previous, (because the base does not translate w.r.t world) but
        // the base rotates w.r.t world. So I have to apply the current rotation to have a correct trajectory
        //world_T_base_.translation() = Eigen::Vector3d::Zero();
        //gait_generator_->setTrajectoryTransformation(world_T_base_);

        base_height_ = base_position_(2);
    }

    void calculatePosition(const double& period, const Eigen::Vector3d& base_position)
    {
        base_position_ = base_position;

        base_linear_velocity_(0) = base_linear_velocity_max_ * base_linear_velocity_scale_x_;
        base_linear_velocity_(1) = base_linear_velocity_max_ * base_linear_velocity_scale_y_;
        base_linear_velocity_(2) = base_linear_velocity_max_ * base_linear_velocity_scale_z_;

        // FIXME this should be reported as computed in the hf.
        base_position_ = base_linear_velocity_ * period + base_position_;
    }

    void calculateOrientation(const double& period, const Eigen::Vector3d& base_orientation)
    {
        base_orientation_ = base_orientation;

        base_angular_velocity_(0) = base_angular_velocity_max_ * base_angular_velocity_scale_roll_;
        base_angular_velocity_(1) = base_angular_velocity_max_ * base_angular_velocity_scale_pitch_;
        base_angular_velocity_(2) = base_angular_velocity_max_ * base_angular_velocity_scale_yaw_;

        //base_angular_velocity_ = world_R_hf_.transpose() * base_angular_velocity_;

        const Eigen::Vector3d& world_base_angular_velocity_ = world_R_hf_ * base_angular_velocity_;

        base_orientation_ = world_base_angular_velocity_ * period + base_orientation_;

        // FIXME base_orientation in which frame is? hf?
        base_rotation_reference_ = Eigen::AngleAxisd(base_orientation_(0), Eigen::Vector3d::UnitX())  // ROLL
                                 * Eigen::AngleAxisd(base_orientation_(1), Eigen::Vector3d::UnitY())  // PITCH
                                 * Eigen::AngleAxisd(base_orientation_(2), Eigen::Vector3d::UnitZ()); // YAW
    }

    // Sets
    void setCmd(const unsigned int cmd)   {cmd_ = cmd;}
    void setBasePosition(const Eigen::Vector3d& position)
    {
        base_position_ = position;
    }
    void setBaseOrientation(const Eigen::Vector3d& orientation)
    {
        base_orientation_ = orientation;
    }
    void setBaseVelocityScaleX(const double scale)
    {
        base_linear_velocity_scale_x_ = scale;
    }
    void setBaseVelocityScaleY(const double scale)
    {
        base_linear_velocity_scale_y_ = scale;
    }
    void setBaseVelocityScaleZ(const double scale)
    {
        base_linear_velocity_scale_z_ = scale;
    }
    void setBaseVelocityScaleRoll(const double scale)
    {
        base_angular_velocity_scale_roll_ = scale;
    }
    void setBaseVelocityScalePitch(const double scale)
    {
        base_angular_velocity_scale_pitch_ = scale;
    }
    void setBaseVelocityScaleYaw(const double scale)
    {
        base_angular_velocity_scale_yaw_ = scale;
    }

    void setMaxLinearVelocity(const double max)
    {
        base_linear_velocity_max_ = max;
    }

    void setMaxAngularVelocity(const double max)
    {
        base_angular_velocity_max_ = max;
    }

    void setHipOffset()
    {

        if(!offset_applied_)
        {
            // FIXME
            hf_X_base_hip_offsets_[0] <<0.3,0.2,0.0;
            hf_X_base_hip_offsets_[1] <<0.3,-0.2,0.0;
            hf_X_base_hip_offsets_[2] <<-0.3,0.2,0.0;
            hf_X_base_hip_offsets_[3] <<-0.3,-0.2,0.0;

            for(unsigned int i=0; i<hips_.size(); i++)
            {
                xbot_model_->getPose(gait_generator_->getFeetNames()[i],"base_link",base_T_foot_);
                xbot_model_->getPose(hips_[i],"base_link",base_T_hip_);
                base_X_hip_foot_offsets_[i] = base_T_foot_.translation() - base_T_hip_.translation();
            }
            offset_applied_ = true;
        }
    }

    // Gets
    unsigned int getCmd()  {return cmd_;}
    const Eigen::Matrix3d& getBaseRotationReference() const {return base_rotation_reference_;}
    const double& getStepLength(const std::string& foot_name) {return steps_length_[foot_name];}
    const double& getStepHeading(const std::string& foot_name) {return steps_heading_[foot_name];}
    const double& getStepHeight(const std::string& foot_name) {return steps_height_[foot_name];}
    const double& getBaseHeight() const {return base_height_;}

private:

    // These are modified by the external interface (e.g joypad and dynamic_reconfigure)
    std::atomic<unsigned int> cmd_;
    std::atomic<double>  base_linear_velocity_scale_x_;
    std::atomic<double>  base_linear_velocity_scale_y_;
    std::atomic<double>  base_linear_velocity_scale_z_;
    std::atomic<double>  base_angular_velocity_scale_roll_;
    std::atomic<double>  base_angular_velocity_scale_pitch_;
    std::atomic<double>  base_angular_velocity_scale_yaw_;
    std::atomic<double>  base_linear_velocity_max_;
    std::atomic<double>  base_angular_velocity_max_;

    Eigen::Vector3d base_linear_velocity_;
    Eigen::Vector3d base_angular_velocity_;

    Eigen::Vector3d base_position_;
    Eigen::Vector3d base_orientation_;

    Eigen::Vector3d hf_delta_hip_;
    Eigen::Vector3d hf_delta_heding_;
    Eigen::Vector3d hf_X_hip_;
    Eigen::Vector3d world_delta_hip_;
    Eigen::Vector3d world_X_hip_;
    Eigen::Vector3d world_X_hip_foot_;
    Eigen::Vector3d world_delta_foot_;
    Eigen::Vector3d world_X_hip_foot_offset_;

    Eigen::Matrix3d world_R_base_;
    Eigen::Matrix3d hf_R_base_;

    bool offset_applied_;

    double base_height_;
    typedef std::map<std::string,double> map_t;
    map_t steps_length_;
    map_t steps_heading_;
    map_t steps_height_;
    Eigen::Matrix3d base_rotation_reference_;

    GaitGenerator::Ptr gait_generator_;
    XBot::ModelInterface::Ptr xbot_model_;

    Eigen::Affine3d world_T_foot_, world_T_hip_, world_T_base_, base_T_hip_, hip_T_foot_, base_T_foot_;
    std::vector<std::string> hips_;
    std::vector<Eigen::Vector3d> base_X_hip_foot_offsets_;
    std::vector<Eigen::Vector3d> hf_X_base_hip_offsets_;

    double yaw_base_;
    Eigen::Matrix3d world_R_hf_;


};

} // namespace


#endif
