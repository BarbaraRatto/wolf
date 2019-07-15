#ifndef PUBLISHERS_H
#define PUBLISHERS_H

#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Core>
#include <type_traits>

namespace wb_controller
{


class RealTimePublisherInterface
{
public:

    /** Initialize the real time publisher. */
    RealTimePublisherInterface(){}

    virtual ~RealTimePublisherInterface(){}

    /** Publish the topic. */
    virtual void publish() = 0;

    inline std::string getTopic(){return topic_name_;}

protected:

    std::string topic_name_;
};

template <typename data_t, typename msg_t>
class RealTimePublisherBase : public RealTimePublisherInterface
{
public:

    typedef realtime_tools::RealtimePublisher<msg_t> rt_publisher_t;

    RealTimePublisherBase(const ros::NodeHandle& ros_nh, const std::string topic_name, data_t* const data)
    {
        // Checks
        assert(topic_name.size() > 0);
        assert(data);
        topic_name_ = topic_name;
        pub_ptr_.reset(new rt_publisher_t(ros_nh,topic_name,10));
        data_ = data;
    }

    /** Publish the topic. */
    inline virtual void publish() = 0;

    inline const data_t* getDataPtr(){if(data_) return data_;}

    inline rt_publisher_t* getPubPtr(){if(pub_ptr_) return pub_ptr_.get();}

protected:
    std::shared_ptr<rt_publisher_t > pub_ptr_;
    data_t* data_ = NULL;

};

template <bool Predicate, typename Result = void>
class TEnableIf;

template <typename Result>
class TEnableIf<true, Result>
{
public:
    typedef Result Type;
};

template<typename data_t> struct IsEigen : std::is_base_of<Eigen::MatrixBase<typename std::decay<data_t>::type>, typename std::decay<data_t>::type > { };

template <typename data_t>
class RealTimePublisher;

template <typename data_t>
inline typename TEnableIf<IsEigen<data_t>::value, void>::Type resize_imp(RealTimePublisher<data_t>* obj)
{
    obj->getPubPtr()->msg_.data.resize(obj->getDataPtr()->size());
}

template <typename data_t>
inline typename TEnableIf<!IsEigen<data_t>::value, void>::Type resize_imp(RealTimePublisher<data_t>* obj)
{
}

template <typename data_t>
inline typename TEnableIf<IsEigen<data_t>::value, void>::Type publish_imp(RealTimePublisher<data_t>* obj)
{
    if(obj->getPubPtr()->trylock() && obj->getDataPtr())
    {
        for(unsigned int i = 0; i < obj->getDataPtr()->size(); i++)
        {
            obj->getPubPtr()->msg_.data[i] = obj->getDataPtr()->operator[](i);
            obj->getPubPtr()->unlockAndPublish();
        }
    }
}

template <typename data_t>
inline typename TEnableIf<!IsEigen<data_t>::value, void>::Type publish_imp(RealTimePublisher<data_t>* obj)
{
    ROS_WARN("Publisher for non Eigen types not defined yet!");
}

template <typename data_t>
class RealTimePublisher : public RealTimePublisherBase<data_t,std_msgs::Float64MultiArray>
{
public:

    RealTimePublisher(const ros::NodeHandle& ros_nh, const std::string topic_name, data_t* const data)
        :RealTimePublisherBase<data_t,std_msgs::Float64MultiArray>(ros_nh,topic_name,data)
    {

        resize_imp<data_t>(this);
        /*if(IsEigen<data_t>::value)
            this->pub_ptr_->msg_.data.resize(this->data_->size());
        else
            this->pub_ptr_->msg_.data.resize(1);*/
    }

    /** Publish the topic. */
    inline void publish() override
    {
        /*if(IsEigen<data_t>::value)
            publish_imp();
        else
            publish_imp();*/

        publish_imp<data_t>(this);
    }


};

class RealTimePublishers
{
public:

    /**
     * @brief Shared pointer to RealTimePublishers
     */
    typedef std::shared_ptr<RealTimePublishers> Ptr;

    /**
     * @brief Shared pointer to const RealTimePublishers
     */
    typedef std::shared_ptr<const RealTimePublishers> ConstPtr;

    typedef RealTimePublisherInterface rt_publisher_interface_t;

    RealTimePublishers(const ros::NodeHandle& ros_nh)
    {
        nh_ = ros_nh;
    }

    // Add a RealTimePublisher already created
    void addPublisher(std::shared_ptr<rt_publisher_interface_t> pub_ptr)
    {
        assert(pub_ptr!=false);
        // Put it into the map with his friends
        map_[pub_ptr->getTopic()] = pub_ptr;
    }

    // Add a new fresh RealTimePublisher
    template <typename data_t>
    void addPublisher(const std::string& topic_name, data_t* const data_ptr)
    {
        std::shared_ptr<RealTimePublisher<data_t>> new_pub_ptr;
        new_pub_ptr.reset(new RealTimePublisher<data_t>(nh_,topic_name,data_ptr));

        //new_pub_ptr->init(nh_,topic_name,data_ptr);

        std::shared_ptr<rt_publisher_interface_t> pub_ptr =
                std::static_pointer_cast<rt_publisher_interface_t>(new_pub_ptr);
        addPublisher(pub_ptr);
    }

    // Publish!
    void publishAll()
    {
        for(pubs_map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
            iterator->second->publish();
    }

protected:

    typedef std::map<std::string,std::shared_ptr<rt_publisher_interface_t> > pubs_map_t;
    typedef typename pubs_map_t::iterator pubs_map_it_t;

    ros::NodeHandle nh_;
    pubs_map_t map_;
};



} // namespace


#endif
