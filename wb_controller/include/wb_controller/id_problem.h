#ifndef ID_PROBLEM_H
#define ID_PROBLEM_H

// OpenSoT
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <OpenSoT/tasks/acceleration/AngularMomentum.h>
#include <OpenSoT/tasks/acceleration/CoM.h>
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

// WB
#include <wb_controller/geometry.h>
#include <wb_controller/utils.h>
#include <wb_controller/ros_wrappers/tasks.h>
#include <wb_controller/quadruped_robot.h>

// STD
#include <atomic>
#include <mutex>
#include <memory>

namespace wb_controller{

/**
 * @brief The IDProblem class wraps the tasks, constraints and the solver used to solve the ID
 */
class IDProblem
{

public:

    const std::string CLASS_NAME = "IDProblem";

    typedef std::shared_ptr<IDProblem> Ptr;

    enum stacks_t {NONE=0,WALKING,MANIPULATION};

    /**
     * @brief IDProblem constructor
     * @param ros node handle
     * @param model pointer to external model
     * @param vector of contact links name
     */
    IDProblem(ros::NodeHandle& nh, QuadrupedRobot::Ptr model);
    ~IDProblem();

    /**
     * @brief solve call this after update()
     * @param x solution form solver (tau)
     * @return true if solved
     */
    bool solve(Eigen::VectorXd& x);

    /**
     * @brief get the contact wrenches, to call after solve()
     */
    const std::vector<Eigen::Vector6d>& getContactWrenches() const;

    /**
     * @brief get the joint accelerations, to call after solve()
     */
    const Eigen::VectorXd& getJointAccelerations() const;

    /**
     * @brief swing with a specific foot i.e. release the contact
     */
    void swingWithFoot(const std::string& foot_name);

    /**
     * @brief stance with a specific foot i.e. activate the contact
     */
    void stanceWithFoot(const std::string& foot_name);

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
     * @brief set the rotation matrix for the friction cones
     * @param R 3d matrix
     */
    void setFrictionConesR(const Eigen::Matrix3d& R);

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
     * @brief set the angular reference and height for the waist (aka base)
     * @param Rot rotation matrix
     * @param z height
     */
    void setWaistReference(const Eigen::Matrix3d &Rot, const double &z);

    /**
     * @brief set the position and velocity reference for the CoM
     * @param position
     * @param velocity
     */
    void setComReference(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity);

    /**
         * @brief Select the stack type to use
         */
    void selectStack(const stacks_t &stack);

    /**
         * @brief Get the current stack type
         */
    unsigned int getCurrentStack();

    /**
         * @brief Switch between WALKING and MANIPULATION stack
         */
    void switchStack();

    /**
     * @brief get the mu parameter for the friction cones
     */
    double getFrictionConesMu() const;

    /**
     * @brief Cartesian tasks
     */
    std::map<std::string,OpenSoT::tasks::acceleration::Cartesian::Ptr> feet_;
    std::map<std::string,OpenSoT::tasks::acceleration::Cartesian::Ptr> arms_;
    OpenSoT::tasks::acceleration::Cartesian::Ptr waistRPY_;
    OpenSoT::tasks::acceleration::Cartesian::Ptr waistZ_;
    OpenSoT::tasks::acceleration::CoM::Ptr com_;
    OpenSoT::tasks::acceleration::AngularMomentum::Ptr angular_momentum_;

    /**
     * @brief Expose the tasks to ROS
     */
    std::map<std::string,TaskRosWrapperInterface::Ptr> tasks_ros_;

    /**
     * @brief postural_ a postural task
     */
    OpenSoT::tasks::acceleration::Postural::Ptr postural_;

    /**
     * @brief qddot_lims_ some bounds
     */
    OpenSoT::constraints::GenericConstraint::Ptr qddot_lims_;

    /**
     * @brief wrenches_lims_ bounds
     */
    OpenSoT::constraints::force::WrenchesLimits::Ptr wrenches_lims_;

    /**
     * @brief _model
     */
    QuadrupedRobot::Ptr model_;

private:

    /**
     * @brief update call after the model.update() to update the autostack
     */
    void update();

    /**
     * @brief dynamics_con_ constraint relates the floating base with the contact forces
     */
    OpenSoT::constraints::TaskToConstraint::Ptr dynamics_con_;

    /**
     * @brief dynamics_task_ constraint relates the floating base with the contact forces
     */
    OpenSoT::tasks::acceleration::DynamicFeasibility::Ptr dynamics_task_;

    /**
     * @brief friction_cones_ constraints
     */
    OpenSoT::constraints::force::FrictionCones::Ptr friction_cones_;

    /**
     * @brief map of stacks
     */
    std::map<unsigned int,OpenSoT::AutoStack::Ptr> stacks_;

    /**
     * @brief solver_ iHQP solver
     */
    std::unique_ptr<OpenSoT::solvers::iHQP> solver_;

    /**
     * @brief id_ inverse dynamics computation & variable helper
     */
    OpenSoT::utils::InverseDynamics::Ptr id_;

    /**
     * @brief x_ full solver solution
     */
    Eigen::VectorXd x_;

    /**
     * @brief qddot_ joint accelerations solution
     */
    Eigen::VectorXd qddot_;

    /**
     * @brief contact_wrenches_ contacts solution
     */
    std::vector<Eigen::Vector6d> contact_wrenches_;

    /**
     * @brief wrench limitis
     */
    Eigen::Vector6d wrench_upper_lims_;
    Eigen::Vector6d wrench_lower_lims_;

    double x_force_lower_lim_;
    double y_force_lower_lim_;
    double z_force_lower_lim_;

    OpenSoT::constraints::force::FrictionCone::friction_cone fc_;

    std::atomic<unsigned int> current_stack_;

    std::mutex solver_lock_;

    std::vector<std::string> foot_names_;
    std::vector<std::string> arm_names_;
    std::vector<std::string> contact_names_;

    Eigen::Affine3d tmp_affine3d_;

};



}

#endif // ID_PROBLEM_H

