#ifndef ABOT_BRINGUP_BASE_DIRVER_H_
#define ABOT_BRINGUP_BASE_DIRVER_H_

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include "base_driver_config.h"

#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include "abot_bringup/transport.h"
#include "abot_bringup/dataframe.h"
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace abot {

class BaseDriver {
 private:
  BaseDriver();

 public:
  static BaseDriver* Instance() {
    if (instance == NULL) instance = new BaseDriver();

    return instance;
  }
  ~BaseDriver();
  void work_loop();

 private:
  void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd);
  void init_cmd_odom();
  void init_pid_debug();
  void init_imu();
  void read_param();

  void update_param();
  void update_odom();
  void update_speed();
  void update_pid_debug();
  void update_imu();

 public:
  BaseDriverConfig& getBaseDriverConfig() {
    return bdg;
  }

  ros::NodeHandle* getNodeHandle() {
    return &nh;
  }

  ros::NodeHandle* getPrivateNodeHandle() {
    return &pn;
  }

 private:
  static BaseDriver* instance;

  BaseDriverConfig bdg;
  boost::shared_ptr<Transport> trans;
  boost::shared_ptr<Dataframe> frame;

  ros::Subscriber cmd_vel_sub;

  ros::Publisher odom_pub;

  nav_msgs::Odometry odom;
  geometry_msgs::TransformStamped odom_trans;
  tf::TransformBroadcaster odom_broadcaster;

  ros::NodeHandle nh;
  ros::NodeHandle pn;

#define MAX_MOTOR_COUNT 4
  ros::Publisher pid_debug_pub_input[MAX_MOTOR_COUNT];
  ros::Publisher pid_debug_pub_output[MAX_MOTOR_COUNT];

  std_msgs::Int32 pid_debug_msg_input[MAX_MOTOR_COUNT];
  std_msgs::Int32 pid_debug_msg_output[MAX_MOTOR_COUNT];

  bool need_update_speed;

  // imu
  bool use_accelerometer, use_gyroscope, use_magnetometer;
  bool use_mag_msg;

  bool perform_calibration, is_calibrated;
  int calibration_samples;
  std::map<std::string, double> acceleration_bias, gyroscope_bias;

  // Covariance
  double linear_acc_stdev, angular_vel_stdev, magnetic_field_stdev;
  boost::array<double, 9> linear_acc_covar;
  boost::array<double, 9> angular_vel_covar;
  boost::array<double, 9> magnetic_field_covar;

  // Used for mag scaling
  double mag_x_min, mag_x_max;  //  [T]
  double mag_y_min, mag_y_max;
  double mag_z_min, mag_z_max;

  // imu pub/sub
  ros::Publisher imu_pub;
  ros::Publisher mag_pub;

  // imu services
  ros::ServiceServer imu_cal_srv;

  bool calibrateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  void fillRowMajor(boost::array<double, 9>& covar, double stdev);
};

}  // namespace abot

#endif
