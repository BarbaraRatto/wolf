#ifndef ID_PROBLEM_H
#define ID_PROBLEM_H

// OpenSoT
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <OpenSoT/tasks/acceleration/CoM.h>
#include <OpenSoT/tasks/MinimizeVariable.h>
#include <OpenSoT/tasks/acceleration/DynamicFeasibility.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/utils/InverseDynamics.h>
#include <OpenSoT/constraints/force/FrictionCone.h>
#include <OpenSoT/constraints/force/WrenchLimits.h>
#include <OpenSoT/constraints/TaskToConstraint.h>

// ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <wb_controller/problemConfig.h>

// WB
#include <wb_controller/geometry.h>
#include <wb_controller/utils.h>
#include <wb_controller/task_ros_wrapper.h>

// STD
#include <atomic>

namespace OpenSoT{

/**
 * @brief The IDProblem class wraps the tasks, constraints and the solver used to solve the ID
 */
class IDProblem
{

public:

    typedef std::shared_ptr<IDProblem> Ptr;

    /**
     * @brief IDProblem constructor
     * @param ros node handle
     * @param model pointer to external model
     * @param vector of contact links name
     */
    IDProblem(ros::NodeHandle& nh, XBot::ModelInterface::Ptr model, std::vector<std::string> feet_names, std::string arm_tip_name = std::string());
    ~IDProblem();

    /**
     * @brief solve call this after update()
     * @param x solution form solver (tau)
     * @return true if solved
     */
    bool solve(Eigen::VectorXd& x);

    /**
     * @brief get the ground reaction forces call this after solve()
     * @param grfs from the solver
     */
    void getGroundReactionForces(Eigen::VectorXd& grfs);

    /**
     * @brief update call after the model.update() to update the autostack
     * @param x input state q
     */
    void update();

    /**
     * @brief reset the tasks
     */
    void reset();

    /**
     * @brief update the tasks with the new external references (i.e. the references coming from the interactive markers)
     * @param task name to update
     */
    void updateReference(const std::string& task_name);

    /**
     * @brief publish the ros topics related to the tasks
     * @param ros current time
     */
    void publish(const ros::Time& time);

    /**
     * @brief set the mu parameter for the friction cones
     * @param mu value
     */
    void setFrictionConesMu(const double& mu);

    /**
     * @brief set the lower bound for the wrench limits along the selected axis (w.r.t world)
     * @param lower bound values
     */
    void setLowerForceBound(const double& x_force,const double& y_force,const double& z_force);

    /**
     * @brief setLowerForceBoundX
     * @param force
     */
    void setLowerForceBoundX(const double& force);

    /**
     * @brief setLowerForceBoundY
     * @param force
     */
    void setLowerForceBoundY(const double& force);

    /**
     * @brief setLowerForceBoundZ
     * @param force
     */
    void setLowerForceBoundZ(const double& force);

    /**
     * @brief set the weight for the forces minimization task
     * @param weight
     */
    void setMinFsWeight(const double& weight);

    /**
     * @brief set the weight for the waist task
     * @param weight
     */
    void setWaistRPYWeight(const double& weight);

    /**
     * @brief set the weight for the postural task
     * @param weight
     */
    void setPosturalWeight(const double& weight);

    /**
     * @brief set the weight for the contacts tasks
     * @param weight
     */
    void setFeetWeight(const double& weight);

    /**
         * @brief Ros dynamic reconfigure callback
         */
    void dynamicReconfigureCallback(wb_controller::problemConfig &config, uint32_t level);

    /**
     * @brief Cartesian tasks
     */
    std::map<std::string,tasks::acceleration::Cartesian::Ptr> _feet;
    tasks::acceleration::Cartesian::Ptr _arm;
    tasks::acceleration::Cartesian::Ptr _waistRPY;
    tasks::acceleration::Cartesian::Ptr _waistZ;
    tasks::acceleration::CoM::Ptr _com;

    /**
     * @brief Forces minimization tasks
     */
    std::vector<tasks::MinimizeVariable::Ptr> _minfs;

    /**
     * @brief Expose the tasks to ROS
     */
    std::map<std::string,TaskRosWrapperInterface::Ptr> _tasks_ros;

    /**
     * @brief _posture a postural task
     */
    tasks::acceleration::Postural::Ptr _postural;

    /**
     * @brief _x_lims some bounds
     */
    constraints::GenericConstraint::Ptr _qddot_lims;

    /**
     * @brief wrench bounds
     */
    constraints::force::WrenchesLimits::Ptr _wrenches_lims;

    /**
     * @brief _model
     */
    XBot::ModelInterface::Ptr _model;

private:

    /**
         * @brief Update the dynamic reconfigure interface
         */
    void dynamicReconfigureUpdate();

    /**
     * @brief _dynamics_con constraint relates the floating base with the contact forces
     */
    constraints::TaskToConstraint::Ptr _dynamics_con;

    /**
     * @brief _dynamics_task constraint relates the floating base with the contact forces
     */
    tasks::acceleration::DynamicFeasibility::Ptr _dynamics_task;

    /**
     * @brief _friction_cones constraints
     */
    constraints::force::FrictionCones::Ptr _friction_cones;

    /**
     * @brief the final ID stack
     */
    AutoStack::Ptr _stack;

    /**
     * @brief _solver iHQP solver
     */
    solvers::iHQP::Ptr _solver;

    /**
     * @brief _id inverse dynamics computation & variable helper
     */
    OpenSoT::utils::InverseDynamics::Ptr _id;

    /**
     * @brief _x decision variables
     */
    Eigen::VectorXd _x;

    Eigen::VectorXd _qddot;
    std::vector<Eigen::Vector6d> _contact_wrenches;

    Eigen::Vector6d _wrench_upper_lims;
    Eigen::Vector6d _wrench_lower_lims;

    /** @brief ROS dynamic reconfigure */
    std::shared_ptr<dynamic_reconfigure::Server<wb_controller::problemConfig>> server_;
    /** @brief ROS dynamic reconfigure config struct */
    wb_controller::problemConfig default_config_;

    std::atomic<double> _mu;
    std::atomic<double> _x_force_lower_lim;
    std::atomic<double> _y_force_lower_lim;
    std::atomic<double> _z_force_lower_lim;

    unsigned int idx_grfs_start_;

};



}

#endif // ID_PROBLEM_H

