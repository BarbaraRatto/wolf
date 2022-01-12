#include <gtest/gtest.h>
#include <ros/ros.h>
#include "wolf_controller/cartesian_trajectory.h"
#include "test_common_utils.h"

static double _period = 0.001;
static wolf_controller::CartesianTrajectory::Ptr _cartesian_trajectory;

// TEST CASES
TEST(CartesianTrajectory, Constructor)
{
  EXPECT_NO_THROW(_cartesian_trajectory = std::make_shared<wolf_controller::CartesianTrajectory>());
}

TEST(CartesianTrajectory, EvaluateSingleWaypoint)
{
  double tot_time = 6.0;
  Eigen::Affine3d T_ref, T_res;
  T_ref.translation() << 0.0, 2.0, 3.0;
  T_ref.linear() = Eigen::Matrix3d::Identity();
  _cartesian_trajectory->setWayPoint(T_ref,tot_time);

  double time = 0.0;
  while(_cartesian_trajectory->getState() == wolf_controller::CartesianTrajectory::state_t::EXECUTING)
  {
    time = _cartesian_trajectory->getTime();
    _cartesian_trajectory->update(_period);
  }
  _cartesian_trajectory->getReference(T_res);
  EXPECT_NEAR( T_res.translation().x() , T_ref.translation().x(), EPS );
  EXPECT_NEAR( T_res.translation().y() , T_ref.translation().y(), EPS );
  EXPECT_NEAR( T_res.translation().z() , T_ref.translation().z(), EPS );
  EXPECT_NEAR(time,tot_time,EPS);
}

TEST(CartesianTrajectory, EvaluateMultipleWaypoints)
{
  double time_1, time_2, time_3;
  time_1 = 2.0;
  time_2 = 5.0;
  time_3 = 10.0;

  Eigen::Affine3d T_res, T_ref1, T_ref2, T_ref3;
  std::vector<wolf_controller::CartesianTrajectory::WayPoint> wps;
  wolf_controller::CartesianTrajectory::WayPoint wp;

  T_ref1.translation() << 0.0, 2.0, 3.0;
  T_ref2.translation() << 5.0, 2.0, 3.0;
  T_ref3.translation() << 5.0, 2.0, 8.0;

  wp.T_ref = T_ref1;
  wp.duration = time_1;
  wps.push_back(wp);

  wp.T_ref = T_ref2;
  wp.duration = time_2;
  wps.push_back(wp);

  wp.T_ref = T_ref3;
  wp.duration = time_3;
  wps.push_back(wp);

  _cartesian_trajectory->setWayPoints(wps);

  double time = 0.0;
  while(_cartesian_trajectory->getState() == wolf_controller::CartesianTrajectory::state_t::EXECUTING)
  {
    time = _cartesian_trajectory->getTime();
    _cartesian_trajectory->update(_period);

    if(time <= time_1 + EPS && time >= time_1 - EPS)
    {
      _cartesian_trajectory->getReference(T_res);
      EXPECT_NEAR( T_ref1.translation().x() , T_res.translation().x(), EPS );
      EXPECT_NEAR( T_ref1.translation().y() , T_res.translation().y(), EPS );
      EXPECT_NEAR( T_ref1.translation().z() , T_res.translation().z(), EPS );
    }
    if(time <= time_2 + EPS && time >= time_2 - EPS)
    {
      _cartesian_trajectory->getReference(T_res);
      EXPECT_NEAR( T_ref2.translation().x() , T_res.translation().x(), EPS );
      EXPECT_NEAR( T_ref2.translation().y() , T_res.translation().y(), EPS );
      EXPECT_NEAR( T_ref2.translation().z() , T_res.translation().z(), EPS );
    }
    if(time <= time_3 + EPS && time >= time_3 - EPS)
    {
      _cartesian_trajectory->getReference(T_res);
      EXPECT_NEAR( T_ref3.translation().x() , T_res.translation().x(), EPS );
      EXPECT_NEAR( T_ref3.translation().y() , T_res.translation().y(), EPS );
      EXPECT_NEAR( T_ref3.translation().z() , T_res.translation().z(), EPS );
    }
  }
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "test_trajectory");
  testing::InitGoogleTest(&argc, argv);


  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
