/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef ROS_WRAPPERS_CARTESIAN_H
#define ROS_WRAPPERS_CARTESIAN_H

// OpenSoT
#include <OpenSoT/Task.h>
#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <OpenSoT/tasks/acceleration/CoM.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/tasks/acceleration/AngularMomentum.h>

// ROS
#include <interactive_markers/menu_handler.h>
#include <urdf/model.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseArray.h>

// WoLF
#include <wolf_controller/ros_wrappers/interface.h>

// WoLF utils
#include <wolf_controller_utils/geometry.h>
#include <wolf_controller_utils/converters.h>

// WoLF msgs
#include <wolf_msgs/CartesianTask.h>
#include <wolf_msgs/Cartesian.h>

// STD
#include <numeric>

// CARTESIAN
class Cartesian : public OpenSoT::tasks::acceleration::Cartesian, public TaskRosWrapperInterface<wolf_msgs::CartesianTask>
{

  template <class Marker_Type>
    inline void EigenAffine3dToVisualizationPose(const Eigen::Affine3d& Frame, Marker_Type& Marker)
    {
        Marker.pose.position.x = Frame.translation().x();
        Marker.pose.position.y = Frame.translation().y();
        Marker.pose.position.z = Frame.translation().z();
        Eigen::Quaterniond q(Frame.linear());
        Marker.pose.orientation.x = q.x();
        Marker.pose.orientation.y = q.y();
        Marker.pose.orientation.z = q.z();
        Marker.pose.orientation.w = q.w();
    }

    template <class Marker_Type>
    inline void PoseToVisualizationPose(const geometry_msgs::Pose& Frame, Marker_Type& Marker)
    {
        Marker.pose.position.x = Frame.position.x;
        Marker.pose.position.y = Frame.position.y;
        Marker.pose.position.z = Frame.position.z;
        Marker.pose.orientation.x = Frame.orientation.x;
        Marker.pose.orientation.y = Frame.orientation.y;
        Marker.pose.orientation.z = Frame.orientation.z;
        Marker.pose.orientation.w = Frame.orientation.w;
    }

public:

  typedef std::shared_ptr<Cartesian> Ptr;

  Cartesian(ros::NodeHandle& nh,
            const std::string task_id,
            const XBot::ModelInterface& robot,
            const std::string& distal_link,
            const std::string& base_link,
            const OpenSoT::AffineHelper& qddot,
            const bool use_mesh = true);

  virtual void registerReconfigurableVariables() override;

  virtual void loadParams() override;

  virtual void updateCost(const Eigen::VectorXd& x) override;

  virtual void publish(const ros::Time& time) override;

  virtual bool reset() override;

protected:

  void makeMarker(const std::string &distal_link, const std::string &base_link, unsigned int interaction_mode, bool show);

  void createInteractiveMarkerControl(const double qw, const double qx, const double qy, const double qz,
                                      const unsigned int interaction_mode);

  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  void referenceCallback(const wolf_msgs::Cartesian::ConstPtr& msg);

  visualization_msgs::InteractiveMarkerControl& makeSTLControl(visualization_msgs::InteractiveMarker& msg);

  visualization_msgs::Marker makeSphere(visualization_msgs::InteractiveMarker& msg);

  visualization_msgs::Marker makeSTL( visualization_msgs::InteractiveMarker &msg );

  Eigen::Affine3d getPose(const std::string& base_link, const std::string& distal_link);

  void makeMenu();

  void changeBaseLink(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::string new_base_link);

  void sendWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void resetMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void resetLastWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void resetAllWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void wayPointCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, double T);

  void publishWP(const std::vector<geometry_msgs::Pose>& wps);

  bool setTrj(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool setContinuous(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool clearMarker(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool spawnMarker(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  void setContinuousCtrl(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

private:

  virtual void _update(const Eigen::VectorXd& x) override;

  /**
   * @brief waypoints_pub_ ROS publisher with the waypoint poses
   */
  ros::Publisher waypoints_pub_;

  /**
   * @brief is_continuous_ true if the poses are directly given to the task
   */
  bool is_continuous_;

  /**
   * @brief waypoints_ contains all the waypoints BUT not the initial position!
   */
  std::vector<geometry_msgs::Pose> waypoints_;

  /**
   * @brief T_ contains the times of each waypoint-trajectory
   */
  std::vector<float> T_;

  int control_type_;

  std_srvs::EmptyRequest req_;
  std_srvs::EmptyResponse res_;

  /**
   * @brief Marker variables
   */
  visualization_msgs::InteractiveMarker interactive_marker_;
  interactive_markers::InteractiveMarkerServer interactive_marker_server_;
  visualization_msgs::Marker marker_;
  interactive_markers::MenuHandler menu_handler_;
  interactive_markers::MenuHandler::EntryHandle reset_marker_entry_;
  interactive_markers::MenuHandler::EntryHandle way_point_entry_;
  interactive_markers::MenuHandler::EntryHandle base_link_entry_;
  interactive_markers::MenuHandler::EntryHandle T_entry_;
  interactive_markers::MenuHandler::EntryHandle T_last_;
  interactive_markers::MenuHandler::EntryHandle reset_lastway_point_entry_;
  interactive_markers::MenuHandler::EntryHandle reset_all_way_points_entry_;
  interactive_markers::MenuHandler::EntryHandle send_way_points_entry_;
  interactive_markers::MenuHandler::EntryHandle continuous_control_entry_;
  visualization_msgs::InteractiveMarkerControl  menu_control_;
  std::map<std::string,interactive_markers::MenuHandler::EntryHandle> map_link_entries_;

  /**
   * @brief urdf_ model description of the robot
   */
  urdf::ModelInterface urdf_;

  /**
   * @brief links_ urdf available links
   */
  std::vector<urdf::LinkSharedPtr> links_;

  /**
   * @brief use_mesh_ true if the end-effector mesh is used for the marker visualization
   */
  bool use_mesh_;

  /**
   * @brief listener_ TF listener
   */
  tf::TransformListener listener_;

};

#endif // ROS_WRAPPERS_CARTESIAN_H

