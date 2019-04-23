#ifndef PUBLISHERS_H
#define PUBLISHERS_H

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


} // namespace


#endif
