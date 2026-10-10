#ifndef ABOT_BRINGUP_BASE_DRIVER_CONFIG_H_
#define ABOT_BRINGUP_BASE_DRIVER_CONFIG_H_

#include <ros/ros.h>

#define USE_DYNAMIC_RECONFIG
#ifdef USE_DYNAMIC_RECONFIG
#include <dynamic_reconfigure/server.h>
#include "abot_bringup/abot_driverConfig.h"
#endif

namespace abot {

class RobotParameter;
class BaseDriverConfig {
 public:
  BaseDriverConfig(ros::NodeHandle& p);
  ~BaseDriverConfig();

  void init(RobotParameter* r);
  void SetRobotParameters();

#ifdef USE_DYNAMIC_RECONFIG
  void dynamic_callback(abot_bringup::abot_driverConfig& config, uint32_t level);
  bool get_param_update_flag();

 private:
  dynamic_reconfigure::Server<abot_bringup::abot_driverConfig> server;
  dynamic_reconfigure::Server<abot_bringup::abot_driverConfig>::CallbackType f;
#endif
 public:
  RobotParameter* rp;

  std::string port;
  int32_t baudrate;
  int32_t  imu_type;
  std::string base_frame;
  std::string odom_frame;

  bool publish_odom_tf;

  bool use_imu;

  std::string cmd_vel_topic;
  std::string odom_topic;

  int32_t freq;

  bool out_pid_debug_enable;

 private:
#ifdef USE_DYNAMIC_RECONFIG
  bool param_update_flag;
#endif
  ros::NodeHandle& pn;
  ros::ServiceClient client;

  bool set_flag;
};

}  // namespace abot

#endif
