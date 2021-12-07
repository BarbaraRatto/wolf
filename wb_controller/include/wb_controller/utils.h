#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>
#include <assert.h>
#include <rt_logger/rt_logger.h>
#include <XBotInterface/TypedefAndEnums.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <wb_controller/quadruped_robot.h>
#include <wb_controller/filters.h>

namespace wb_controller
{
#define ROBOT_REAL
#define GRAVITY 9.81
//#define REACHING_MOTION
#define FLOATING_BASE_DOFS 6
#define N_LEGS 4 // Fixed number of legs supported
#define N_ARMS 1 // Fixed number of arms supported
#define THREADS_SLEEP_TIME_ms 4
#define THROTTLE_SEC 3.0
#define COMPUTE_COST
#define EPS 0.001 //std::numeric_limits<double>::epsilon()
extern double _period;

// If I use closed loop trajectory and remove the floating base velocity estimation, there is no movement at all! the robot
// stays in the same position because the feet don't move relatively to the base anymore. There is no reset!
//#define OPEN_LOOP_TRAJECTORY

inline void affine3dToPose(const Eigen::Affine3d& affine3d, geometry_msgs::Pose& pose)
{
    // Translation
    pose.position.x = affine3d.translation().x();
    pose.position.y = affine3d.translation().y();
    pose.position.z = affine3d.translation().z();
    // Note unfortunately affine3d can not be converted in quaternion directly...
    // Rotation
    pose.orientation.x = static_cast<Eigen::Quaterniond>(affine3d.linear()).x();
    pose.orientation.y = static_cast<Eigen::Quaterniond>(affine3d.linear()).y();
    pose.orientation.z = static_cast<Eigen::Quaterniond>(affine3d.linear()).z();
    pose.orientation.w = static_cast<Eigen::Quaterniond>(affine3d.linear()).w();
}

inline void vector3dToPosePosition(const Eigen::Vector3d& vector3d, geometry_msgs::Pose& pose)
{
    // Translation
    pose.position.x = vector3d.x();
    pose.position.y = vector3d.y();
    pose.position.z = vector3d.z();
}

inline void quaterniondToPoseOrientation(const Eigen::Quaterniond& quaterniond, geometry_msgs::Pose& pose)
{
    // Rotation
    pose.orientation.x = quaterniond.x();
    pose.orientation.y = quaterniond.y();
    pose.orientation.z = quaterniond.z();
    pose.orientation.w = quaterniond.w();
}

inline void vector6dToTwist(const Eigen::Vector6d& vector6d, geometry_msgs::Twist& twist)
{
    twist.linear.x  = vector6d(0);
    twist.linear.y  = vector6d(1);
    twist.linear.z  = vector6d(2);
    twist.angular.x = vector6d(3);
    twist.angular.y = vector6d(4);
    twist.angular.z = vector6d(5);
}

inline void vector3dToVector3(const Eigen::Vector3d& vector3d, geometry_msgs::Vector3& vector3)
{
    vector3.x = vector3d.x();
    vector3.y = vector3d.y();
    vector3.z = vector3d.z();
}

// NOTE: by default we use the same leg order as RBDL (alphabetic order)
extern std::vector<std::string> _dof_names;
extern std::vector<std::string> _cartesian_names;
extern std::vector<std::string> _xyz;
extern std::vector<std::string> _rpy;
extern std::vector<std::string> _joints_prefix;
extern std::vector<std::string> _legs_prefix;
enum _leg_id {LF=0,LH,RF,RH};
inline std::vector<std::string> sortByLegPrefix(const std::vector<std::string>& names, const std::vector<std::string>& order = {"lf","lh","rf","rh"} )
{
    // Sort the names following order
    assert(names.size() == N_LEGS);
    assert(order.size() == N_LEGS);
    std::vector<std::string> ordered_names(N_LEGS);
    for(unsigned int i=0;i<names.size();i++)
        for(unsigned int j=0;j<order.size();j++)
            if(names[i].find(order[j]) != std::string::npos)
                ordered_names[j] = names[i];

    return ordered_names;
}

inline QuadrupedRobot* createRobotModel(ros::NodeHandle& root_nh)
{
  // Create the quadruped robot object, it wraps the xbot model with some meta information
  std::string urdf, srdf;
  if(!root_nh.getParam("/robot_description",urdf)) // Get the robot description from the global namespace "/"
  {
      throw std::runtime_error("No robot_description given in namespace /");
  }
  if(!root_nh.getParam("/robot_semantic_description",srdf)) // Get the robot semantic description from the global namespace "/"
  {
      throw std::runtime_error("No robot_semantic_description given in namespace /");
  }

  return new wb_controller::QuadrupedRobot(urdf,srdf);
}

class Trigger
{
public:
    Trigger()
    {
        old_value = false;
    }
    bool update(const bool& value)
    {
        bool res = (value && old_value != value ? true : false);
        old_value = value;
        return res;
    }
private:
    bool old_value;
};


class AxisToTrigger
{

public:
    enum status_t {UP=0,DOWN,STEADY};

    AxisToTrigger()
    {
        axis_old_value_ = 0.0;
    }

    void update(const double axis)
    {
        status_ = STEADY;
        if(std::abs(axis)>0.0 && axis_old_value_!=axis)
        {
            if(axis>=1.0)
                status_ = UP;
            else if (axis<=-1.0)
                status_ = DOWN;
        }

        axis_old_value_ = axis;
    }

    status_t getStatus()
    {
        return status_;
    }

private:
    double axis_old_value_;
    status_t status_;

};

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


} // namespace

#endif
