/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef COMMON_H
#define COMMON_H

#include <string>
#include <vector>
#include <assert.h>
#include <rt_logger/rt_logger.h>
#include <wolf_controller/quadruped_robot.h>

namespace wolf_controller
{
#define ODOM_FRAME "odom"
#define BASE_FOOTPRINT_FRAME "base_footprint"
#define BASE_STABILIZED_FRAME "base_stabilized"
#define ANGULAR_VELOCITIES_WRT_BASE // Comment it if the IMU's velocities are defined wrt world
#define GRAVITY 9.81 // Gravity value
//#define REACHING_MOTION
#define FLOATING_BASE_DOFS 6
#define N_LEGS 4 // Fixed number of legs supported
#define N_ARMS 1 // Fixed number of arms supported
#define THREADS_SLEEP_TIME_ms 4
#define THROTTLE_SEC 3.0
//#define COMPUTE_COST
//#define DEBUG
#define EPS 0.00001 //std::numeric_limits<double>::epsilon()
extern double _period;
extern std::string _robot_name;
extern std::string _robot_model_name;
extern std::string _tf_prefix;
extern std::string _rt_gui_group;
#define TOPIC( data ) (_robot_name+"/wolf_controller/"#data)
//#define OPEN_LOOP_TRAJECTORY

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

} // namespace

#endif
