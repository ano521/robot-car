#include "base_driver.h"
#include "data_holder.h"

#include <std_msgs/Float32MultiArray.h>
#include "serial_transport.h"
#include "simple_dataframe_master.h"
#include <boost/assign/list_of.hpp>

namespace abot {

BaseDriver* BaseDriver::instance = NULL;

static double GRAVITY = -9.81;                       // [m/s/s]
static double MILIGAUSS_TO_TESLA_SCALE = 0.0000001;  // From Milligauss [mG] to Tesla [T]

BaseDriver::BaseDriver()
  : pn("~")
  , bdg(pn)
  , use_accelerometer(true)
  , use_gyroscope(true)
  , use_magnetometer(true)
  , use_mag_msg(true)
  , perform_calibration(true)
  , is_calibrated(false) {
  // init config
  bdg.init(&DataHolder::get()->parameter);

  trans = boost::make_shared<SerialTransport>(bdg.port, bdg.baudrate);

  frame = boost::make_shared<SimpleDataframe>(trans.get());

  ROS_INFO("BaseDriver startup...");
  if (trans->init()) {
    ROS_INFO("connected to main board");
  } else {
    ROS_ERROR("oops!!! can't connect to main board, please check the usb connection or baudrate!");
    return;
  }

  frame->init();

  for (int i = 0; i < 3; i++) {
    if (frame->interact(ID_GET_VERSION)) break;
    ros::Duration(1).sleep();  // wait for device
  }

  ROS_INFO("robot version:%s build time:%s", DataHolder::get()->firmware_info.version, DataHolder::get()->firmware_info.time);

  init_cmd_odom();

  init_pid_debug();

  read_param();

  if (bdg.use_imu) {
    init_imu();
  }
}

BaseDriver::~BaseDriver() {
  if (instance != NULL) delete instance;
}

void BaseDriver::init_cmd_odom() {
  frame->interact(ID_INIT_ODOM);

  ROS_INFO_STREAM("subscribe cmd topic on [" << bdg.cmd_vel_topic << "]");
  cmd_vel_sub = nh.subscribe(bdg.cmd_vel_topic, 1000, &BaseDriver::cmd_vel_callback, this);

  ROS_INFO_STREAM("advertise odom topic on [" << bdg.odom_topic << "]");
  odom_pub = nh.advertise<nav_msgs::Odometry>(bdg.odom_topic, 50);

  // init odom_trans
  odom_trans.header.frame_id = bdg.odom_frame;
  odom_trans.child_frame_id = bdg.base_frame;

  odom_trans.transform.translation.z = 0;

  // init odom
  odom.header.frame_id = bdg.odom_frame;
  odom.pose.pose.position.z = 0.0;
  odom.child_frame_id = bdg.base_frame;
  odom.twist.twist.linear.y = 0;

  if (!bdg.publish_odom_tf) {
    odom.pose.covariance = boost::assign::list_of(1e-3)(0)(0)(0)(0)(0)(0)(1e-3)(0)(0)(0)(0)(0)(0)(1e6)(0)(0)(0)(0)(0)(0)(1e6)(0)(0)(0)(0)(0)(0)(1e6)(
        0)(0)(0)(0)(0)(0)(1e3);

    odom.twist.covariance = boost::assign::list_of(1e-3)(0)(0)(0)(0)(0)(0)(1e-3)(0)(0)(0)(0)(0)(0)(1e6)(0)(0)(0)(0)(0)(0)(1e6)(0)(0)(0)(0)(0)(0)(1e6)(
        0)(0)(0)(0)(0)(0)(1e3);
  }

  need_update_speed = false;
}

void BaseDriver::init_pid_debug() {
  if (bdg.out_pid_debug_enable) {
    const char* input_topic_name[MAX_MOTOR_COUNT] = {"motor1_input", "motor2_input", "motor3_input", "motor4_input"};
    const char* output_topic_name[MAX_MOTOR_COUNT] = {"motor1_output", "motor2_output", "motor3_output", "motor4_output"};
    for (size_t i = 0; i < MAX_MOTOR_COUNT; i++) {
      pid_debug_pub_input[i] = nh.advertise<std_msgs::Int32>(input_topic_name[i], 1000);
      pid_debug_pub_output[i] = nh.advertise<std_msgs::Int32>(output_topic_name[i], 1000);
    }
  }
}

void BaseDriver::init_imu() {
  pn.param<bool>("imu/use_accelerometer", use_accelerometer, use_accelerometer);
  pn.param<bool>("imu/use_gyroscope", use_gyroscope, use_gyroscope);
  pn.param<bool>("imu/use_magnetometer", use_magnetometer, use_magnetometer);
  pn.param<bool>("imu/perform_calibration", perform_calibration, perform_calibration);

  // 创建一个服务,用于接收运行中的校准请求, 可以rosservice命令查看
  imu_cal_srv = nh.advertiseService("imu/calibrate_imu", &BaseDriver::calibrateCallback, this);

  // 是否使能加速度计或陀螺仪
  if (use_accelerometer || use_gyroscope) {
    // 根据配置决定是否进行校准
    if (!pn.getParam("imu/accelerometer_bias", acceleration_bias) || !pn.getParam("imu/gyroscope_bias", gyroscope_bias)) {
      ROS_WARN("IMU calibration NOT found.");
      is_calibrated = false;
    } else {
      ROS_INFO("IMU calibration found.");
      pn.getParam("imu/accelerometer_bias", acceleration_bias);
      pn.getParam("imu/gyroscope_bias", gyroscope_bias);
      is_calibrated = true;
    }

    // 校准采样点数
    pn.param<int>("imu/calibration_samples", calibration_samples, 500);

    // 转换为ROS标准的sensor_msgs::Imu类型的topic
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

    // 线加速度协方差
    pn.param<double>("imu/linear_acc_stdev", linear_acc_stdev, 0.0);
    fillRowMajor(linear_acc_covar, linear_acc_stdev);

    // 角速度协方差
    pn.param<double>("imu/angular_vel_stdev", angular_vel_stdev, 0.0);
    fillRowMajor(angular_vel_covar, angular_vel_stdev);
  }

  // 是否使能磁力计
  if (use_magnetometer) {
    // Magnetometer calibration values.
    pn.param<double>("mag/x/min", mag_x_min, -0.000078936);
    pn.param<double>("mag/x/max", mag_x_max, 0.000077924);
    pn.param<double>("mag/y/min", mag_y_min, -0.000075532);
    pn.param<double>("mag/y/max", mag_y_max, 0.000076360);
    pn.param<double>("mag/z/min", mag_z_min, -0.000079948);
    pn.param<double>("mag/z/max", mag_z_max, 0.000064216);

    pn.param<bool>("imu/use_mag_msg", use_mag_msg, use_mag_msg);

    if (use_mag_msg) {
      // 转换为ROS标准的sensor_msgs::MagneticField类型的topic
      mag_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 5);

      pn.param<double>("imu/magnetic_field_stdev", magnetic_field_stdev, 0.0);
      fillRowMajor(magnetic_field_covar, magnetic_field_stdev);
    } else {
      // 使用Vector3Stamped类型的topic
      mag_pub = nh.advertise<geometry_msgs::Vector3Stamped>("imu/mag", 5);
    }
  }
}

void BaseDriver::read_param() {
  RobotParameter* param = &DataHolder::get()->parameter;
  memset(param, 0, sizeof(RobotParameter));

  frame->interact(ID_GET_ROBOT_PARAMTER);

  DataHolder::dump_params(param);

  bdg.SetRobotParameters();
}

void BaseDriver::cmd_vel_callback(const geometry_msgs::Twist& vel_cmd) {
  ROS_INFO_STREAM("cmd_vel:[" << vel_cmd.linear.x << " " << vel_cmd.linear.y << " " << vel_cmd.angular.z << "]");

  DataHolder::get()->velocity.v_liner_x = vel_cmd.linear.x * 100;
  DataHolder::get()->velocity.v_liner_y = vel_cmd.linear.y * 100;
  DataHolder::get()->velocity.v_angular_z = vel_cmd.angular.z * 100;

  need_update_speed = true;
}

void BaseDriver::work_loop() {
  ros::Rate loop(bdg.freq);
  while (ros::ok()) {
    update_param();

    update_odom();

    update_pid_debug();

    update_speed();

    if (bdg.use_imu) {
      if (DataHolder::get()->parameter.imu_type == IMU_TYPE_GY65 || DataHolder::get()->parameter.imu_type == IMU_TYPE_GY85 ||
          DataHolder::get()->parameter.imu_type == IMU_TYPE_GY87) {
        update_imu();
      }
    }

    loop.sleep();

    ros::spinOnce();
  }
}

void BaseDriver::update_param() {
#ifdef USE_DYNAMIC_RECONFIG
  if (bdg.get_param_update_flag()) {
    frame->interact(ID_SET_ROBOT_PARAMTER);
    ros::Rate loop(5);
    loop.sleep();
  }
#endif
}

void BaseDriver::update_odom() {
  frame->interact(ID_GET_ODOM);

  ros::Time current_time = ros::Time::now();

  float x = DataHolder::get()->odom.x * 0.01;
  float y = DataHolder::get()->odom.y * 0.01;
  float th = DataHolder::get()->odom.yaw * 0.01;

  float vxy = DataHolder::get()->odom.v_liner_x * 0.01;
  float vth = DataHolder::get()->odom.v_angular_z * 0.01;

  // ROS_INFO("odom: x=%.2f y=%.2f th=%.2f vxy=%.2f vth=%.2f", x, y ,th, vxy,vth);

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  // publish_odom_tf
  if (bdg.publish_odom_tf) {
    odom_trans.header.stamp = current_time;
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);
  }

  // publish the message
  odom.header.stamp = current_time;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation = odom_quat;
  odom.twist.twist.linear.x = vxy;
  odom.twist.twist.angular.z = vth;

  odom_pub.publish(odom);
}

void BaseDriver::update_speed() {
  if (need_update_speed) {
    ROS_INFO_STREAM("update_speed");
    need_update_speed = !(frame->interact(ID_SET_VELOCITY));
  }
}

void BaseDriver::update_pid_debug() {
  if (bdg.out_pid_debug_enable) {
    frame->interact(ID_GET_PID_DATA);

    for (size_t i = 0; i < MAX_MOTOR_COUNT; i++) {
      pid_debug_msg_input[i].data = DataHolder::get()->pid_data.input[i];
      pid_debug_msg_output[i].data = DataHolder::get()->pid_data.output[i];

      pid_debug_pub_input[i].publish(pid_debug_msg_input[i]);
      pid_debug_pub_output[i].publish(pid_debug_msg_output[i]);
    }
  }
}

void BaseDriver::update_imu() {
  frame->interact(ID_GET_IMU_DATA);

  if (!use_accelerometer) {
    ROS_ERROR_ONCE("Accelerometer not found!");
  }
  if (!use_gyroscope) {
    ROS_ERROR_ONCE("Gyroscope not found!");
  }
  if (!use_magnetometer) {
    ROS_ERROR_ONCE("Magnetometer not found!");
  }

  if (perform_calibration || !is_calibrated) {
    ROS_WARN_ONCE("Calibrating accelerometer and gyroscope, make sure robot is stationary and level.");

    static int taken_samples;

    if (taken_samples < calibration_samples) {
      acceleration_bias["x"] += DataHolder::get()->imu_data[0];
      acceleration_bias["y"] += DataHolder::get()->imu_data[1];
      acceleration_bias["z"] += DataHolder::get()->imu_data[2];

      gyroscope_bias["x"] += DataHolder::get()->imu_data[3];
      gyroscope_bias["y"] += DataHolder::get()->imu_data[4];
      gyroscope_bias["z"] += DataHolder::get()->imu_data[5];

      taken_samples++;
    } else {
      acceleration_bias["x"] /= calibration_samples;
      acceleration_bias["y"] /= calibration_samples;
      acceleration_bias["z"] = acceleration_bias["z"] / calibration_samples + GRAVITY;

      gyroscope_bias["x"] /= calibration_samples;
      gyroscope_bias["y"] /= calibration_samples;
      gyroscope_bias["z"] /= calibration_samples;

      ROS_INFO("Calibrating accelerometer and gyroscope complete.");
      ROS_INFO("Bias values can be saved for reuse.");
      ROS_INFO("Accelerometer: x: %f, y: %f, z: %f", acceleration_bias["x"], acceleration_bias["y"], acceleration_bias["z"]);
      ROS_INFO("Gyroscope: x: %f, y: %f, z: %f", gyroscope_bias["x"], gyroscope_bias["y"], gyroscope_bias["z"]);

      nh.setParam("imu/accelerometer_bias", acceleration_bias);
      nh.setParam("imu/gyroscope_bias", gyroscope_bias);

      is_calibrated = true;
      perform_calibration = false;
      taken_samples = 0;
    }
  } else {
    if (use_accelerometer || use_gyroscope) {
      sensor_msgs::ImuPtr imu_msg = boost::make_shared<sensor_msgs::Imu>();
      imu_msg->header.stamp = ros::Time::now();
      imu_msg->header.frame_id = bdg.base_frame;

      imu_msg->angular_velocity.x = DataHolder::get()->imu_data[3] - gyroscope_bias["x"];
      imu_msg->angular_velocity.y = DataHolder::get()->imu_data[4] - gyroscope_bias["y"];
      imu_msg->angular_velocity.z = DataHolder::get()->imu_data[5] - gyroscope_bias["z"];
      imu_msg->orientation_covariance = angular_vel_covar;

      imu_msg->linear_acceleration.x = DataHolder::get()->imu_data[0] - acceleration_bias["x"];
      imu_msg->linear_acceleration.y = DataHolder::get()->imu_data[1] - acceleration_bias["y"];
      imu_msg->linear_acceleration.z = DataHolder::get()->imu_data[2] - acceleration_bias["z"];
      imu_msg->linear_acceleration_covariance = linear_acc_covar;

      imu_pub.publish(imu_msg);
    }

    if (use_magnetometer) {
      if (use_mag_msg) {
        sensor_msgs::MagneticFieldPtr mag_msg = boost::make_shared<sensor_msgs::MagneticField>();
        mag_msg->header.stamp = ros::Time::now();
        mag_msg->header.frame_id = bdg.base_frame;

        mag_msg->magnetic_field.x = (DataHolder::get()->imu_data[6] * MILIGAUSS_TO_TESLA_SCALE - (mag_x_max - mag_x_min) / 2 - mag_x_min);
        mag_msg->magnetic_field.y = (DataHolder::get()->imu_data[7] * MILIGAUSS_TO_TESLA_SCALE - (mag_y_max - mag_y_min) / 2 - mag_y_min);
        mag_msg->magnetic_field.z = (DataHolder::get()->imu_data[8] * MILIGAUSS_TO_TESLA_SCALE - (mag_z_max - mag_z_min) / 2 - mag_z_min);
        mag_msg->magnetic_field_covariance = magnetic_field_covar;

        mag_pub.publish(mag_msg);
      } else {
        geometry_msgs::Vector3StampedPtr mag_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
        mag_msg->header.stamp = ros::Time::now();
        mag_msg->header.frame_id = bdg.base_frame;

        mag_msg->vector.x = (DataHolder::get()->imu_data[6] - (mag_x_max - mag_x_min) / 2 - mag_x_min) * MILIGAUSS_TO_TESLA_SCALE;
        mag_msg->vector.y = (DataHolder::get()->imu_data[7] - (mag_y_max - mag_y_min) / 2 - mag_y_min) * MILIGAUSS_TO_TESLA_SCALE;
        mag_msg->vector.z = (DataHolder::get()->imu_data[8] - (mag_z_max - mag_z_min) / 2 - mag_z_min) * MILIGAUSS_TO_TESLA_SCALE;

        mag_pub.publish(mag_msg);
      }
    }
  }
}

bool BaseDriver::calibrateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  ROS_WARN("Calibrating accelerometer and gyroscope, make sure robot is stationary and level.");
  perform_calibration = true;
  return true;
}

void BaseDriver::fillRowMajor(boost::array<double, 9>& covar, double stdev) {
  std::fill(covar.begin(), covar.end(), 0.0);
  covar[0] = pow(stdev, 2);  // X(roll)
  covar[4] = pow(stdev, 2);  // Y(pitch)
  covar[8] = pow(stdev, 2);  // Z(yaw)
}

}  // namespace abot
