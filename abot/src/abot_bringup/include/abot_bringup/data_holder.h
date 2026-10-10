#ifndef ABOT_BRINGUP_DATA_HOLDER_H_
#define ABOT_BRINGUP_DATA_HOLDER_H_

#include <string.h>

namespace abot {

#pragma pack(1)

typedef int int32;
typedef short int16;
typedef unsigned short uint16;

struct RobotFirmware {
  char version[16];  // 固件版本
  char time[16];     // 构建时间
};

enum IMU_TYPE {
  IMU_TYPE_GY65 = 49,
  IMU_TYPE_GY85 = 69,
  IMU_TYPE_GY87 = 71
};

enum MODEL_TYPE {
  MODEL_TYPE_2WD_DIFF = 1,
  MODEL_TYPE_4WD_DIFF = 2,
  MODEL_TYPE_3WD_OMNI = 101,
  MODEL_TYPE_4WD_OMNI = 102,
  MODEL_TYPE_4WD_MECANUM = 201,
};

#define MOTOR_ENCODER_1_FLAG 0x01
#define MOTOR_ENCODER_2_FLAG 0x02
#define MOTOR_ENCODER_3_FLAG 0x04
#define MOTOR_ENCODER_4_FLAG 0x08

struct RobotParameter {
  union {
    char buff[64];
    struct {
      unsigned short wheel_diameter;      // 轮子直径  mm
      unsigned short wheel_track;         // 差分：轮距， 三全向轮：直径，四全向：前后轮距+左右轮距 mm
      unsigned short encoder_resolution;  // 编码器分辨率
      unsigned char do_pid_interval;      // pid间隔 (ms)
      unsigned short kp;
      unsigned short ki;
      unsigned short kd;
      unsigned short ko;                       // pid参数比例
      unsigned short cmd_last_time;            // 命令持久时间ms 超过该时间会自动停止运动
      unsigned short max_v_liner_x;            // 最大x线速度
      unsigned short max_v_liner_y;            // 最大y线速度
      unsigned short max_v_angular_z;          // 最大角速度
      unsigned char imu_type;                  // imu类型 参见IMU_TYPE
      unsigned short motor_ratio;              // 电机减速比
      unsigned char model_type;                // 运动模型类型 参见MODEL_TYPE
      unsigned char motor_nonexchange_flag;    // 电机标志参数        1 正接      0 反接(相当于电机线交换)
      unsigned char encoder_nonexchange_flag;  // 编码器标志参数      1 正接      0 反接(相当于编码器ab相交换)
    };
  };
};

struct RobotVelocity {
  short v_liner_x;    // 线速度 前>0 cm/s
  short v_liner_y;    // 差分轮 为0  cm/s
  short v_angular_z;  // 角速度 左>0 0.01rad/s  100 means 1 rad/s
};

struct RobotOdom {
  short v_liner_x;    // 线速度 前>0 后<0  cm/s
  short v_liner_y;    // 差分轮 为0        cm/s
  short v_angular_z;  // 角速度 左>0 右<0  0.01rad/s   100 means 1 rad/s
  int32 x;            // 里程计坐标x       cm (这里long为4字节，下同)
  int32 y;            // 里程计坐标y       cm
  short yaw;          // 里程计航角        0.01rad     100 means 1 rad
};

struct RobotPidData {
  int32 input[4];   // 各轮子驱动输入值
  int32 output[4];  // 个轮子输出值
};

struct RobotImu {
  union {
    float imu_data[9];

    struct {
      float ax;  // 加速度x m2/s
      float ay;  // 加速度y m2/s
      float az;  // 加速度z m2/s
      float gx;  // 角速度x rad/s
      float gy;  // 角速度y rad/s
      float gz;  // 角速度z rad/s
      float mx;  // 磁力x mGA
      float my;  // 磁力y mGA
      float mz;  // 磁力z mGA
    } imu;
  };
};

#pragma pack(0)

class DataHolder {
 public:
  static DataHolder* get() {
    static DataHolder dh;
    return &dh;
  }

  void load_parameter();

  void save_parameter();

  static void dump_params(struct RobotParameter* params);

 private:
  DataHolder() {
    memset(&firmware_info, 0, sizeof(struct RobotFirmware));
    memset(&parameter, 0, sizeof(struct RobotParameter));
    memset(&velocity, 0, sizeof(struct RobotVelocity));
    memset(&odom, 0, sizeof(struct RobotOdom));
    memset(&pid_data, 0, sizeof(struct RobotPidData));
    memset(&imu_data, 0, sizeof(imu_data));
  }

 public:
  struct RobotFirmware firmware_info;
  struct RobotParameter parameter;
  struct RobotVelocity velocity;
  struct RobotOdom odom;
  struct RobotPidData pid_data;

  float imu_data[9];
};

}  // namespace abot

#endif
