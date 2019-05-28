#include <gtest/gtest.h>
#include <ros/ros.h>

static std::string ns;
static std::string service;
static unsigned int cnt = 0;
static unsigned int cnt_limit = 100;

bool isControllerAlive()
{
  // Check if there is a publisher/subscriber in the controller.
  if(ros::service::waitForService(service,1.0)) // Wait 1 sec
    return true;
  else
    return false;
}

// TEST CASES
TEST(DlsControllerTest, test)
{
  // wait for the controller to be alive
  while(!isControllerAlive() && cnt++<cnt_limit)
  {
    ros::Duration(0.1).sleep();
  }
  ASSERT_LE(cnt,cnt_limit);
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "dls_controller_test");
  testing::InitGoogleTest(&argc, argv);

  ns = ros::this_node::getNamespace();
  service = ns+"/dls_controller/set_parameters";

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
