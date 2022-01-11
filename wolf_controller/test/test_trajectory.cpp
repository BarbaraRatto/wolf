#include <gtest/gtest.h>
#include <ros/ros.h>
#include "wolf_controller/cartesian_trajectory.h"
#include "test_common_utils.h"

static double _period = 0.001;
static double _tot_time = 6.0;
static wolf_controller::CartesianTrajectory::Ptr _cartesian_trajectory;

// TEST CASES
TEST(CartesianTrajectory, Constructor)
{
  EXPECT_NO_THROW(_cartesian_trajectory = std::make_shared<wolf_controller::CartesianTrajectory>());
}

TEST(CartesianTrajectory, Evaluate)
{
  Eigen::Affine3d T_ref;
  T_ref.translation() << 0.0, 2.0, 3.0;
  T_ref.linear() = Eigen::Matrix3d::Identity();
  _cartesian_trajectory->setWayPoint(T_ref,5.0);

  double time = 0.0;
  while(time < _tot_time)
  {
    _cartesian_trajectory->update(time,_period);
    time+=_period;
  }
  _cartesian_trajectory->getReference(T_ref);
  EXPECT_NEAR( T_ref.translation().x() , 0.0, EPS );
  EXPECT_NEAR( T_ref.translation().y() , 2.0, EPS );
  EXPECT_NEAR( T_ref.translation().z() , 3.0, EPS );
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
