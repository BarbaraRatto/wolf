#ifndef ROS_WRAPPERS_INTERFACE_H
#define ROS_WRAPPERS_INTERFACE_H

// ROS
#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
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

protected:

    std::shared_ptr<ros::AsyncSpinner> spinner_;
    std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> server_;
};

class TaskRosWrapperInterface : public RosWrapperInterface
{

public:

  struct {
      std::atomic<bool> set_ext_lambda    = true;
      std::atomic<bool> set_ext_weight    = true;
      std::atomic<bool> set_ext_gains     = true;
      std::atomic<bool> set_ext_reference = false;
  } OPTIONS;

    typedef std::shared_ptr<TaskRosWrapperInterface> Ptr;

    TaskRosWrapperInterface() {}
    virtual ~TaskRosWrapperInterface(){}

    /**
     * @brief setExternalLambda set the external lambda 1 and 2 to the task, RT-SAFE
     */
    virtual void setExternalLambda() {}
    /**
     * @brief getExternalLambda, RT-SAFE
     * @return return the lambda created from the dynamic reconfigure interface
     */
    virtual void getExternalLambda(double& /*lambda1*/, double& /*lambda2*/) {}
    /**
     * @brief setExternalReference set the external reference to the task, RT-SAFE
     */
    virtual void setExternalReference() {}
    /**
     * @brief getExternalReference, RT-SAFE
     * @return return the reference created by the interactive marker
     */
    virtual Eigen::Affine3d& getExternalReference() {}
    /**
     * @brief setExternalGains set the external gains to the task, RT-SAFE
     */
    virtual void setExternalGains() {}
    /**
     * @brief getExternalGains, RT-SAFE
     * @return return the gains created by the dynamic reconfigure interface
     */
    template<typename Derived>
    void getExternalGains(Eigen::MatrixBase<Derived>& /*Kp*/, Eigen::MatrixBase<Derived>& /*Kd*/) {}
    /**
     * @brief getExternalKp, RT-SAFE
     * @return return the Kp gain
     */
    template<typename Derived>
    Eigen::MatrixBase<Derived>& getExternalKp() {}
    /**
     * @brief getExternalKd, RT-SAFE
     * @return return the Kd gain
     */
    template<typename Derived>
    Eigen::MatrixBase<Derived>& getExternalKd() {}

    /**
     * @brief update performs the sets from NON-RT to RT, RT-SAFE
     */
    virtual void update() = 0;

    virtual void reset() {};

 protected:

    Eigen::VectorXd       tmp_vectorxd_;
    Eigen::Affine3d       tmp_affine3d_;
    Eigen::Vector6d       tmp_vector6d_;
    Eigen::Vector3d       tmp_vector3d_;
    Eigen::Quaterniond    tmp_quaterniond_;

    realtime_tools::RealtimeBuffer<Eigen::Affine3d> rt_pose_reference_;

    std::atomic<double> rt_lambda1_;
    std::atomic<double> rt_lambda2_;
    std::atomic<double> rt_weight_diag_;

    std::atomic<double> rt_kp_x_;
    std::atomic<double> rt_kp_y_;
    std::atomic<double> rt_kp_z_;
    std::atomic<double> rt_kp_roll_;
    std::atomic<double> rt_kp_pitch_;
    std::atomic<double> rt_kp_yaw_;

    std::atomic<double> rt_kd_x_;
    std::atomic<double> rt_kd_y_;
    std::atomic<double> rt_kd_z_;
    std::atomic<double> rt_kd_roll_;
    std::atomic<double> rt_kd_pitch_;
    std::atomic<double> rt_kd_yaw_;

    std::string task_id_;

};

#endif // ROS_WRAPPERS_INTERFACE_H

