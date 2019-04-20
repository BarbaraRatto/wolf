#ifndef UTILS_H
#define UTILS_H

#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>

namespace dls_controller
{

template <typename T>
class RealTimePublisherVector
{
public:

    /** Initialize the real time publisher. */
    RealTimePublisherVector(const ros::NodeHandle& ros_nh, const std::string topic_name)
    {
        // Checks
        assert(topic_name.size() > 0);
        topic_name_ = topic_name;
        pub_ptr_.reset(new rt_publisher_t(ros_nh,topic_name,10));

    }
    /** Publish the topic. */
    //inline void Publish(const Eigen::Ref<const Eigen::VectorXd>& in)
    inline void Publish(const T& in)
    {
        if(pub_ptr_ && pub_ptr_->trylock())
        {
            //pub_ptr_->msg_.header.stamp = ros::Time::now();
            int data_size = pub_ptr_->msg_.data.size();
            //assert(data_size >= in.size());
            for(int i = 0; i < data_size; i++)
            {
                pub_ptr_->msg_.data[i] = in(i);
            }
            pub_ptr_->unlockAndPublish();
        }
    }

    /** Remove an element in the vector. */
    inline void Remove(const int idx)
    {
        if(pub_ptr_)
        {
            pub_ptr_->lock();
            pub_ptr_->msg_.data.erase(pub_ptr_->msg_.data.begin()+idx);
            pub_ptr_->unlock();
        }
    }

    /** Resize the vector. */
    inline void Resize(const int dim)
    {
        if(pub_ptr_)
        {
            pub_ptr_->lock();
            pub_ptr_->msg_.data.resize(dim);
            pub_ptr_->unlock();
        }
    }

    /** Add a new element at the back. */
    inline void PushBackEmpty()
    {
        if(pub_ptr_)
        {
            pub_ptr_->lock();
            pub_ptr_->msg_.data.push_back(0.0);
            pub_ptr_->unlock();
        }
    }

    inline std::string getTopic(){return topic_name_;}

private:
    typedef realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> rt_publisher_t;
    std::string topic_name_;
    boost::shared_ptr<rt_publisher_t > pub_ptr_;
};

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

    void setDutyCycle(double duty_cycle)
    {
        assert(duty_cycle > 0 && duty_cycle <=1);
        dc_ = duty_cycle;
    }

    void update(const double& period, const bool& contact, const bool& trigger_swing)
    {

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


class GaitGenerator
{
public:
    GaitGenerator(const double& duty_cycle, const std::vector<std::string>& feet_names, const std::string& gait_type)
    {
        assert(feet_names.size==4);// We assume we are working with a dog
        for(unsigned int i = 0; i<feet_names.size(); i++)
        {
            feet_[feet_names[i]].reference = Eigen::Affine3d::Identity();
            feet_[feet_names[i]].initial_pose = Eigen::Affine3d::Identity();
            feet_[feet_names[i]].scheduler = FootScheduler(duty_cycle);
            feet_[feet_names[i]].is_in_contact = true;

            // FIXME
            feet_[feet_names[i]].amp = 0.1;
            feet_[feet_names[i]].swing_frequency = 1.5;
            feet_[feet_names[i]].time = 0.0;
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
        return feet_[foot_name].reference;
    }

    bool isSwinging(const std::string& foot_name)
    {
        return feet_[foot_name].scheduler.isSwing();
    }

    void setContact(const std::string& foot_name, const bool& contact)
    {
        feet_[foot_name].is_in_contact = contact;
    }

    void setInitialPose(const std::string& foot_name, const Eigen::Affine3d& initial_pose)
    {
        feet_[foot_name].initial_pose = initial_pose;
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

        // Set the initial value to each foot
        for(feet_t::iterator it = feet_.begin(); it != feet_.end(); it++)
            it->second.reference = it->second.initial_pose;

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
                // Compute the periodic swing FIXME move it into the trajectory generator
                feet_[selected_feet_[i]].reference.translation().z() +=
                        feet_[selected_feet_[i]].amp/2.0 * (0.8 - std::cos(2.0 * M_PI * (feet_[selected_feet_[i]].swing_frequency * feet_[selected_feet_[i]].time)));
                feet_[selected_feet_[i]].time += period;
            }
            else
            {
                feet_[selected_feet_[i]].time = 0.0;
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
        Eigen::Affine3d reference;
        Eigen::Affine3d initial_pose;
        bool is_in_contact;

        // The following should go in a different class
        double amp;
        double time;
        double swing_frequency;
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
