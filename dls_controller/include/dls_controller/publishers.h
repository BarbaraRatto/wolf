#ifndef PUBLISHERS_H
#define PUBLISHERS_H

#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Core>

namespace dls_controller
{

void EigenToRosJointState(const Eigen::VectorXd& position,
                          const Eigen::VectorXd& velocity,
                          const Eigen::VectorXd& effort,
                          const std::vector<std::string>& names,
                          sensor_msgs::JointState& joint_state)
{

    assert(position.size() == velocity.size() == effort.size() == names.size());
    assert(joint_state.position.size() == position.size());
    assert(joint_state.velocity.size() == velocity.size());
    assert(joint_state.effort.size() == effort.size());

    for(unsigned int i = 0; i < names.size(); i++)
    {
        joint_state.name[i] = names[i];
        joint_state.position[i] = position(i);
        joint_state.velocity[i] = velocity(i);
        joint_state.effort[i] = effort(i);
    }
}

class RealTimePublisherInterface
{
public:

    /** Initialize the real time publisher. */
    RealTimePublisherInterface()
    {
        initialized_ = false;
    }

    virtual ~RealTimePublisherInterface(){}

    /** Publish the topic. */
    virtual void publish(const ros::Time& time) = 0;

    inline std::string getTopic(){return topic_name_;}

protected:
    std::string topic_name_;
    bool initialized_ = false;
};

template <typename data_t>
class RealTimePublisherBase : public RealTimePublisherInterface
{
public:

    typedef realtime_tools::RealtimePublisher<data_t> rt_publisher_t;

    /** Initialize the real time publisher. */
    /*RealTimePublisherBase(const ros::NodeHandle& ros_nh, const std::string topic_name, data_t* const data)
    {
        init(ros_nh,topic_name,data);
    }*/

    inline void init(const ros::NodeHandle& ros_nh, const std::string topic_name, data_t* const data)
    {
        // Checks
        assert(topic_name.size() > 0);
        assert(data);
        topic_name_ = topic_name;
        pub_ptr_.reset(new rt_publisher_t(ros_nh,topic_name,10));
        data_ = data;
        initialized_ = true;
    }

    /** Publish the topic. */
    inline virtual void publish(const ros::Time& time) = 0;

    inline data_t* getDataPtr(){if(data_) return data_;}

    inline std::string getTopic(){return topic_name_;}

protected:
    std::shared_ptr<rt_publisher_t > pub_ptr_;
    data_t* data_ = NULL;

};

template <typename data_t>
class RealTimePublisher : public RealTimePublisherBase<data_t>
{
public:
        RealTimePublisher(){}
        inline void publish(const ros::Time& time){}
};

template <>
class RealTimePublisher<sensor_msgs::JointState> : public RealTimePublisherBase<sensor_msgs::JointState>
{
public:

    /** Publish the topic. */
    inline void publish(const ros::Time& time) override
    {
        if(pub_ptr_->trylock() && getDataPtr())
        {
            for(unsigned int i = 0; i < data_->position.size(); i++)
            {
                pub_ptr_->msg_.position[i]  = data_->position[i];
                pub_ptr_->msg_.velocity[i]  = data_->velocity[i];
                pub_ptr_->msg_.effort[i]    = data_->effort[i];
                pub_ptr_->msg_.header.stamp = time;
                pub_ptr_->unlockAndPublish();
            }
        }
    }
};

template <>
class RealTimePublisher<dls_controller::TaskPoses> : public RealTimePublisherBase<dls_controller::TaskPoses>
{
public:

    /** Publish the topic. */
    inline void publish(const ros::Time& time) override
    {
        if(pub_ptr_->trylock() && getDataPtr())
        {

        }
    }
};

class RealTimePublishers
{
public:

    typedef RealTimePublisherInterface rt_publisher_interface_t;

    // Add a RealTimePublisher already created
    void addPublisher(std::shared_ptr<rt_publisher_interface_t> pub_ptr)
    {
        assert(pub_ptr!=false);
        // Put it into the map with his friends
        map_[pub_ptr->getTopic()] = pub_ptr;
    }

    // Add a new fresh RealTimePublisher
    template <typename data_t>
    void addPublisher(const ros::NodeHandle& ros_nh, const std::string topic_name, data_t* const data_ptr)
    {
        std::shared_ptr<RealTimePublisher<data_t> > new_pub_ptr;
        new_pub_ptr.reset(new RealTimePublisher<data_t>());

        new_pub_ptr->init(ros_nh,topic_name,data_ptr);

        std::shared_ptr<rt_publisher_interface_t> pub_ptr =
                std::static_pointer_cast<rt_publisher_interface_t>(new_pub_ptr);
        addPublisher(pub_ptr);
    }

    // Publish!
    void publishAll(const ros::Time& time)
    {
        for(pubs_map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
            iterator->second->publish(time);
    }

protected:

    typedef std::map<std::string,std::shared_ptr<rt_publisher_interface_t> > pubs_map_t;
    typedef typename pubs_map_t::iterator pubs_map_it_t;

    pubs_map_t map_;
};



} // namespace


#endif
