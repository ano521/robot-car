#include "abot_bringup/data_holder.h"
#include <ros/ros.h>

namespace abot {

void DataHolder::dump_params(struct RobotParameter* params) {
  std::string model_name = "UNKNOWN";
  if (params->model_type == MODEL_TYPE_2WD_DIFF) {
    model_name = "2WD_DIFF";
  } else if (params->model_type == MODEL_TYPE_4WD_DIFF) {
    model_name = "4WD_DIFF";
  } else if (params->model_type == MODEL_TYPE_3WD_OMNI) {
    model_name = "3WD_OMNI";
  } else if (params->model_type == MODEL_TYPE_4WD_OMNI) {
    model_name = "4WD_OMNI";
  } else if (params->model_type == MODEL_TYPE_4WD_MECANUM) {
    model_name = "4WD_MECANUM";
  }
  std::string imu_name = "UNKNOWN";
  if (params->imu_type == IMU_TYPE_GY65) {
    imu_name = "GY65";
  } else if (params->imu_type == IMU_TYPE_GY85) {
    imu_name = "GY85";
  } else if (params->imu_type == IMU_TYPE_GY87) {
    imu_name = "GY87";
  }

  ROS_INFO(
      "RobotParameters:\n \
                        \t\tmodel: %s(%d)\n \
                        \t\twheel_diameter: %dmm\n \
                        \t\twheel_track: %dmm\n \
                        \t\tencoder_resolution: 4*%d\n \
                        \t\tmotor_ratio: 1:%d\n\
                        \t\tmotor_exchange_flag: %d\n\
                        \t\tencoder_exchange_flag: %d\n\
                        \t\tpid interval: %dms\n\
                        \t\tkp: %d/%d ki: %d/%d kd: %d/%d\n\
                        \t\tcmd hold time: %dms\n\
                        \t\tvelocity limit, x: %0.2fm/s y: %0.2fm/s a: %0.2frad/s\n\
                        \t\timu: %s(%d)\n \
                         ",
      model_name.c_str(),
      params->model_type,
      params->wheel_diameter,
      params->wheel_track,
      params->encoder_resolution / 4,
      params->motor_ratio,
      params->motor_nonexchange_flag,
      params->encoder_nonexchange_flag,
      params->do_pid_interval,
      params->kp,
      params->ko,
      params->ki,
      params->ko,
      params->kd,
      params->ko,
      params->cmd_last_time,
      params->max_v_liner_x / 100.,
      params->max_v_liner_y / 100.,
      params->max_v_angular_z / 100.,
      imu_name.c_str(),
      params->imu_type);
}
}  // namespace abot
