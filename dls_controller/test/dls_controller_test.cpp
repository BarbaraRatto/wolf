#include <gtest/gtest.h>
#include <ros/ros.h>

bool isControllerAlive()
{
  // Check if there is a publisher/subscriber in the controller.
  return true;
}

// TEST CASES
TEST(DlsControllerTest, test)
{
  // wait for the controller to be alive
  while(!isControllerAlive())
  {
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("Press a key to continue...");
  getchar();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "dls_controller_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Duration(0.5).sleep();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
