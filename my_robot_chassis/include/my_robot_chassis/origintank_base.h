#ifndef ORIGINTANK_BASE__ORIGINTANK_BASE_H_
#define ORIGINTANK_BASE__ORIGINTANK_BASE_H_

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>

#include <fcntl.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

class CCodbotBase : public rclcpp::Node
{
public:
  CCodbotBase();
  bool init();
  void GetParams();
  void SetGorgePro(const char * serial_num);
  int set_serial(int fd, int nSpeed, int nBits, char nEvent, int nStop);
  void DiffCallBack(const geometry_msgs::msg::Twist::SharedPtr cmd_input);
  void AckerCallBack(const geometry_msgs::msg::Twist::SharedPtr cmd_input);
  void MecanumCallBack(const geometry_msgs::msg::Twist::SharedPtr cmd_input);
  static void * MyReadThread(void * pt);
  static void * MyWriteThread(void * th);
  void ReceiveUpdate(int data_len, unsigned char * data_type, unsigned char new_buff);
  bool CheckRawData(unsigned char * raw_data, int data_num, int current_code);
  void tf_pub_callback();
  void PublishUltrasonicRange(uint16_t raw_distance);

public:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;

  std::string cmd_port;
  std::string cmd_topic;
  std::string imu_topic;
  std::string imu_frame;
  std::string baud_rate;
  std::string ultrasonic_topic;
  std::string ultrasonic_frame;
  int debug_num {0};
  double coeff_odom_x {1.131};
  double coeff_odom_y {1.077};
  double ultrasonic_min_range_m_ {0.03};
  double ultrasonic_max_range_m_ {4.0};
  double ultrasonic_unit_scale_m_ {0.01};
  double ultrasonic_field_of_view_rad_ {0.26};

  const std::string alltype[4] = {"A100", "D100", "T100", "M100"};
  std::string current_type;
  std::string feedback_type;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

private:
  pthread_t th_base_write {};

  struct BaseReader
  {
    int serialNum {-1};
    unsigned char carVelRaw[13] = {0};
    unsigned char motorVelRaw[12] = {0};
    unsigned char batteryRaw[5] = {0};
    unsigned char imuRpyRaw[13] = {0};
    unsigned char imuRaw[34] = {0};
    unsigned char carTypeRaw[5] = {0};
    unsigned char ultrasonicRaw[6] = {0};

    unsigned char writebuff[13] = {0xcd, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int nread {0};
    int nwrite {0};
    std::atomic_bool recv_update_flag {false};
    std::atomic_bool send_update_flag {false};
  } baseCommunication;

  struct BaseData
  {
    float vx {0.0F};
    float vy {0.0F};
    float yaw {0.0F};
    float v1 {0.0F};
    float v2 {0.0F};
    float v3 {0.0F};
    float v4 {0.0F};
    float init_yaw {0.0F};
    float curr_orien {0.0F};
    float angler_speed {0.0F};
  } currentBaseData;

public:
  std::string odom_frame;
  std::string odom_topic;
  std::string odom_child_frame;

  struct OdomData
  {
    float speed_ratio = 1.2941F;
    float wheel_distance = 0.19F;
    float encode_sampling_time = 0.04F;
    float cmd_vel_linear_max {0.3F};
    float cmd_vel_linear_min {0.0F};
    float cmd_vel_anglar_max {2.0F};
    float cmd_vel_anglar_min {0.0F};
    float cmd_vel_angle_max {25.0F};
    float cmd_vel_angle_min {0.0F};

    float position_x = 0.0F;
    float position_y = 0.0F;
    float twist_x = 0.0F;
    float twist_y = 0.0F;
    float twist_th = 0.0F;
    float oriention = 0.0F;
    bool update_flag = false;
  } myOdomData;
};

#endif  // ORIGINTANK_BASE__ORIGINTANK_BASE_H_
