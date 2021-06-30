#ifndef ROS_WRAPPERS_INTERFACE_H
#define ROS_WRAPPERS_INTERFACE_H

// ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <interactive_markers/interactive_marker_server.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <wb_controller/utils.h>

// STD
#include <atomic>

class RosWrapperInterface
{

public:

    typedef std::shared_ptr<RosWrapperInterface> Ptr;

    RosWrapperInterface(){spinner_.reset(new ros::AsyncSpinner(1)); spinner_->start();}
    virtual ~RosWrapperInterface(){spinner_->stop();}

    virtual void publish(const ros::Time& /*time*/) = 0;
    virtual void updateReference() {}
    virtual void dynamicReconfigureUpdate() {}
    virtual void reset() {}
    virtual void createInteractiveMarker() {}

protected:

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;

    Eigen::Affine3d       tmp_affine3d_;
    Eigen::VectorXd       tmp_vectorxd_;
    Eigen::Vector3d       tmp_vector3d_;
    Eigen::Vector6d       tmp_vector6d_;
    Eigen::Quaterniond    tmp_quaterniond_;

    realtime_tools::RealtimeBuffer<Eigen::Affine3d> rt_affine3d_;

};

#endif // ROS_WRAPPERS_INTERFACE_H

