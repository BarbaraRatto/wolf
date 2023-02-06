#ifndef TEST_COMMON_UTILS_H
#define TEST_COMMON_UTILS_H

#define EIGEN_MALLOC_CHECKS
#ifdef EIGEN_MALLOC_CHECKS
  #define EIGEN_RUNTIME_NO_MALLOC
  #define START_REAL_TIME_CRITICAL_CODE() do { Eigen::internal::set_is_malloc_allowed(false); } while (0)
  #define END_REAL_TIME_CRITICAL_CODE() do { Eigen::internal::set_is_malloc_allowed(true); } while (0)
#else
  #define START_REAL_TIME_CRITICAL_CODE() do {  } while (0)
  #define END_REAL_TIME_CRITICAL_CODE() do {  } while (0)
#endif

#include <ros/ros.h>
#include <Eigen/Dense>
#include <wolf_controller/quadruped_robot.h>
#include <wolf_controller/common.h>
#include <wolf_controller_utils/geometry.h>

#endif
