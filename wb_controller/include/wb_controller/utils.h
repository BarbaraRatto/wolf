#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>
#include <assert.h>
#include <wb_controller/logger.h>
#include <XBotInterface/TypedefAndEnums.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

namespace wb_controller
{

#define FLOATING_BASE_DOFS 6
#define N_LEGS 4
#define DT 0.001 // FIXME
#define THREADS_SLEEP_TIME_ms 4
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

template <typename T>
inline T secondOrderFilter(T& varOutputSecondFilter , T& varOutputFirstFilter , T const& varNew , T const& gain)
{ 
    varOutputFirstFilter = (1- gain) * varOutputFirstFilter + gain * varNew;
    varOutputSecondFilter = (1 - gain) * varOutputSecondFilter + gain * varOutputFirstFilter;
    return varOutputSecondFilter;
} 

enum leg_id {LF=0,RH,RF,LH};
static std::vector<std::string> feet_names_global = {"lf_foot","rh_foot","rf_foot","lh_foot"};

inline std::vector<std::string> sortByLegName(const std::vector<std::string>& names)
{
    // Sort the names following this convention:
    assert(names.size() == N_LEGS);
    std::string lf="lf"; // 0
    std::string rh="rh"; // 1
    std::string rf="rf"; // 2
    std::string lh="lh"; // 3
    std::vector<std::string> ordered_names(N_LEGS);
    for(unsigned int i=0;i<names.size();i++)
    {
        if(names[i].find(lf) != std::string::npos)
            ordered_names[leg_id::LF] = names[i]; //LF
        if(names[i].find(rh) != std::string::npos)
            ordered_names[leg_id::RH] = names[i]; //RH
        if(names[i].find(rf) != std::string::npos)
            ordered_names[leg_id::RF] = names[i]; //RF
        if(names[i].find(lh) != std::string::npos)
            ordered_names[leg_id::LH] = names[i]; //LH
    }
    return ordered_names;
}

} // namespace

#endif
