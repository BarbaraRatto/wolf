#define EIGEN_RUNTIME_NO_MALLOC
#include <gtest/gtest.h>
#include <ctime>
#include "test_common_utils.h"
#include "robot_dummy.h"
#include "wolf_controller/controller.h"
#include <Eigen/src/Core/util/Memory.h>

static wolf_controller::Controller::Ptr _controller_ptr;
static wolf_controller::RobotDummy::Ptr _hw_ptr;
static std::unique_ptr<ros::NodeHandle> _root_nh_ptr;
static std::unique_ptr<ros::NodeHandle> _controller_nh_ptr;
static double _period = 0.001;

// TEST CASES
TEST(ControllerTest, Init)
{
  EXPECT_NO_THROW(_hw_ptr->init());
  ASSERT_TRUE(_controller_ptr->init(_hw_ptr.get(),*_root_nh_ptr,*_controller_nh_ptr));
}

TEST(ControllerTest, Starting)
{
  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(_controller_ptr->starting(ros::Time::now()));
  END_REAL_TIME_CRITICAL_CODE();
}

TEST(ControllerTest, Update)
{
  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(_controller_ptr->update(ros::Time::now(),ros::Duration(_period))); // Update
  _controller_ptr->standUp(true); // Change to Standup
  EXPECT_NO_THROW(_controller_ptr->update(ros::Time::now(),ros::Duration(_period))); // Update
  while(_controller_ptr->getRobotModel()->getState() == wolf_controller::QuadrupedRobot::INIT)  // Running the impedance
    EXPECT_NO_THROW(_controller_ptr->update(ros::Time::now(),ros::Duration(_period))); // Update
  while(_controller_ptr->getRobotModel()->getState() == wolf_controller::QuadrupedRobot::STANDING_UP)  // Standing up
    EXPECT_NO_THROW(_controller_ptr->update(ros::Time::now(),ros::Duration(_period))); // Update
  EXPECT_NO_THROW(_controller_ptr->update(ros::Time::now(),ros::Duration(_period))); // Running the solver
  END_REAL_TIME_CRITICAL_CODE();
}

TEST(ControllerTest, Stopping)
{
  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(_controller_ptr->stopping(ros::Time::now()));
  END_REAL_TIME_CRITICAL_CODE();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_controller");
  testing::InitGoogleTest(&argc, argv);

  _root_nh_ptr = std::make_unique<ros::NodeHandle>();
  _controller_nh_ptr = std::make_unique<ros::NodeHandle>("wolf_controller");
  _hw_ptr = std::make_shared<wolf_controller::RobotDummy>();
  _controller_ptr = std::make_shared<wolf_controller::Controller>();

  //const std::string library_name = class_loader::systemLibraryFormat("wolf_controller");
  //class_loader::ClassLoader loader(library_name, false);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
