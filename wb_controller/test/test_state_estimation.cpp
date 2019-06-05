// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// ros_control
#include <OpenSoT/floating_base_estimation/qp_estimation.h>

OpenSoT::floating_base_estimation::qp_estimation::Ptr _qp_estimation;
XBot::ModelInterface::Ptr _xbot_model;
Eigen::VectorXd _joint_positions(18);
Eigen::VectorXd _joint_velocities(18);
Eigen::VectorXd _qdot(6);


void update(const sensor_msgs::JointState::ConstPtr& msg)
{

    for(unsigned int i =0;i<msg->name.size();i++)
    {
        _joint_positions(i) = msg->position[i];
        _joint_velocities(i) = msg->velocity[i];
    }

    _xbot_model->setJointVelocity(_joint_velocities);
    _xbot_model->setJointPosition(_joint_positions);
    _qp_estimation->update(OpenSoT::FloatingBaseEstimation::Update::All);

   _qp_estimation->getFloatingBaseTwist(_qdot);

   ROS_INFO_STREAM("Estimated FB Vel: "<< _qdot.transpose());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_state_estimation");
    ros::NodeHandle root_nh;

    std::vector<std::string> feet_names;
    feet_names.resize(4);
    feet_names[0] = "lf_foot";
    feet_names[1] = "rf_foot";
    feet_names[2] = "lh_foot";
    feet_names[3] = "rh_foot";

    // Create the ModelInterface from XBot
    XBot::ConfigOptions opt;
    std::string urdf, srdf;

    if(!root_nh.getParam("/robot_description",urdf)) // Get the robot description from the global namespace "/"
    {
        ROS_ERROR_STREAM("No robot_description given in namespace /");
        return false;
    }
    if(!root_nh.getParam("/robot_semantic_description",srdf)) // Get the robot semantic description from the global namespace "/"
    {
        ROS_ERROR_STREAM("No robot_semantic_description given in namespace /");
        return false;
    }
    if(!opt.set_urdf(urdf))
    {
        ROS_ERROR_STREAM("Unable to load urdf");
        return false;
    }
    if(!opt.set_srdf(srdf))
    {
        ROS_ERROR_STREAM("Unable to load srdf");
        return false;
    }
    if(!opt.generate_jidmap())
    {
        ROS_ERROR_STREAM("Unable to load jidmap");
        return false;
    }
    opt.set_parameter("is_model_floating_base", true);
    opt.set_parameter<std::string>("model_type", "RBDL");
    _xbot_model = XBot::ModelInterface::getModel(opt);


    Eigen::Matrix6d contact_matrix; contact_matrix.setZero();
    contact_matrix.block(0,0,3,3) << Eigen::Matrix3d::Identity();
    _qp_estimation.reset(new OpenSoT::floating_base_estimation::qp_estimation(_xbot_model,feet_names,contact_matrix));

    ros::Subscriber sub = root_nh.subscribe("/hyq/joint_states", 1000, update);

    ros::spin();

    return 0;
}
