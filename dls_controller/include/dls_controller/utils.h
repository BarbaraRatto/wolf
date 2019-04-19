#ifndef UTILS_H
#define UTILS_H

#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64MultiArray.h>

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

    FootScheduler(double duty_cycle)
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

    void update(double period, bool contact, bool trigger_swing)
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


} // namespace


#endif
