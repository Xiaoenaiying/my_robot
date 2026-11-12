#include "../include/my_robot_chassis/origintank_base.h"
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <ctime>
#include <iostream>

using namespace std::chrono_literals;

CCodbotBase::CCodbotBase()
: Node("origintank_node")
{
  init();
}

bool CCodbotBase::init()
{
  GetParams();

  if (current_type == alltype[0]) {
    RCLCPP_INFO(get_logger(), "Current chassis type is ackerman");
    cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic, 10, std::bind(&CCodbotBase::AckerCallBack, this, _1));
  } else if (current_type == alltype[1] || current_type == alltype[2]) {
    RCLCPP_INFO(get_logger(), "Current chassis type is differential/tank");
    cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic, 10, std::bind(&CCodbotBase::DiffCallBack, this, _1));
  } else if (current_type == alltype[3]) {
    RCLCPP_INFO(get_logger(), "Current chassis type is mecanum");
    cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic, 10, std::bind(&CCodbotBase::MecanumCallBack, this, _1));
  } else {
    RCLCPP_ERROR(get_logger(), "can not find %s chassis", current_type.c_str());
    return false;
  }

  imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
  odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
  ultrasonic_pub = this->create_publisher<sensor_msgs::msg::Range>(ultrasonic_topic, 10);
  tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  timer_ = this->create_wall_timer(50ms, std::bind(&CCodbotBase::tf_pub_callback, this));

  SetGorgePro(cmd_port.c_str());
  if (baseCommunication.serialNum < 0) {
    return false;
  }

  pthread_t th_base_read {};
  int th_read_status = pthread_create(&th_base_read, nullptr, MyReadThread, this);
  if (0 != th_read_status) {
    RCLCPP_ERROR(get_logger(), "can't create base read thread: %d", th_read_status);
    return false;
  }

  int th_write_status = pthread_create(&th_base_write, nullptr, MyWriteThread, this);
  if (0 != th_write_status) {
    RCLCPP_ERROR(get_logger(), "can't create base write thread: %d", th_write_status);
    return false;
  }

  return true;
}

void CCodbotBase::GetParams()
{
  this->declare_parameter<std::string>("cmd_port", "/dev/ttyUSB0");
  cmd_port = this->get_parameter("cmd_port").as_string();

  this->declare_parameter<std::string>("baud_rate", "B115200");
  baud_rate = this->get_parameter("baud_rate").as_string();

  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  cmd_topic = this->get_parameter("cmd_vel_topic").as_string();

  this->declare_parameter<std::string>("odom_topic", "/odom");
  odom_topic = this->get_parameter("odom_topic").as_string();

  this->declare_parameter<std::string>("odom_frame", "odom");
  odom_frame = this->get_parameter("odom_frame").as_string();

  this->declare_parameter<std::string>("odom_child_frame", "base_footprint");
  odom_child_frame = this->get_parameter("odom_child_frame").as_string();

  this->declare_parameter<std::string>("imu_topic", "/imu");
  imu_topic = this->get_parameter("imu_topic").as_string();

  this->declare_parameter<std::string>("imu_frame", "imu_link");
  imu_frame = this->get_parameter("imu_frame").as_string();

  this->declare_parameter<std::string>("ultrasonic_topic", "/ultrasonic/front/range");
  ultrasonic_topic = this->get_parameter("ultrasonic_topic").as_string();

  this->declare_parameter<std::string>("ultrasonic_frame", "ultrasonic_front_link");
  ultrasonic_frame = this->get_parameter("ultrasonic_frame").as_string();

  this->declare_parameter("ultrasonic_min_range_m", 0.03);
  ultrasonic_min_range_m_ = this->get_parameter("ultrasonic_min_range_m").as_double();

  this->declare_parameter("ultrasonic_max_range_m", 4.0);
  ultrasonic_max_range_m_ = this->get_parameter("ultrasonic_max_range_m").as_double();

  this->declare_parameter("ultrasonic_unit_scale_m", 0.01);
  ultrasonic_unit_scale_m_ = this->get_parameter("ultrasonic_unit_scale_m").as_double();

  this->declare_parameter("ultrasonic_field_of_view_rad", 0.26);
  ultrasonic_field_of_view_rad_ =
    this->get_parameter("ultrasonic_field_of_view_rad").as_double();

  this->declare_parameter("debug_num", 0);
  this->get_parameter("debug_num", debug_num);

  this->declare_parameter("coeff_odom_x", 1.131);
  coeff_odom_x = this->get_parameter("coeff_odom_x").as_double();

  this->declare_parameter("coeff_odom_y", 1.077);
  coeff_odom_y = this->get_parameter("coeff_odom_y").as_double();

  this->declare_parameter<std::string>("current_type", "M100");
  current_type = this->get_parameter("current_type").as_string();

  this->declare_parameter("cmd_vel_linear_max", 0.3);
  myOdomData.cmd_vel_linear_max = this->get_parameter("cmd_vel_linear_max").as_double();

  this->declare_parameter("cmd_vel_linear_min", 0.0);
  myOdomData.cmd_vel_linear_min = this->get_parameter("cmd_vel_linear_min").as_double();

  this->declare_parameter("cmd_vel_anglar_max", 2.0);
  myOdomData.cmd_vel_anglar_max = this->get_parameter("cmd_vel_anglar_max").as_double();

  this->declare_parameter("cmd_vel_anglar_min", 0.0);
  myOdomData.cmd_vel_anglar_min = this->get_parameter("cmd_vel_anglar_min").as_double();

  this->declare_parameter("cmd_vel_angle_max", 25.0);
  myOdomData.cmd_vel_angle_max = this->get_parameter("cmd_vel_angle_max").as_double();

  this->declare_parameter("cmd_vel_angle_min", 0.0);
  myOdomData.cmd_vel_angle_min = this->get_parameter("cmd_vel_angle_min").as_double();
}

void CCodbotBase::SetGorgePro(const char * serial_num)
{
  baseCommunication.serialNum = open(serial_num, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (-1 == baseCommunication.serialNum) {
    RCLCPP_ERROR(get_logger(), "Open %s serial port failed: %s", serial_num, strerror(errno));
    return;
  }

  struct termios my_options;
  if (0 != tcgetattr(baseCommunication.serialNum, &my_options)) {
    RCLCPP_ERROR(get_logger(), "Get serial port attribute failed: %s", strerror(errno));
    close(baseCommunication.serialNum);
    baseCommunication.serialNum = -1;
    return;
  }

  tcflush(baseCommunication.serialNum, TCIFLUSH);
  cfsetispeed(&my_options, B115200);
  cfsetospeed(&my_options, B115200);

  my_options.c_cflag |= (CLOCAL | CREAD);
  my_options.c_cflag &= ~CSIZE;
  my_options.c_cflag |= CS8;
  my_options.c_cflag &= ~PARENB;
  my_options.c_cflag &= ~CSTOPB;
  my_options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  my_options.c_oflag &= ~OPOST;
  my_options.c_iflag &= ~(IXON | IXOFF | IXANY);
  my_options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
  my_options.c_cc[VTIME] = 0;
  my_options.c_cc[VMIN] = 0;

  if (0 != tcsetattr(baseCommunication.serialNum, TCSANOW, &my_options)) {
    RCLCPP_ERROR(get_logger(), "Set %s serial port failed: %s", serial_num, strerror(errno));
    close(baseCommunication.serialNum);
    baseCommunication.serialNum = -1;
    return;
  }

  RCLCPP_INFO(get_logger(), "Serial port %s opened at 115200", serial_num);
}

int CCodbotBase::set_serial(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
  struct termios newttys1, oldttys1;
  if (tcgetattr(fd, &oldttys1) != 0) {
    perror("Setup IMU Serial");
    return -1;
  }

  memset(&newttys1, 0, sizeof(newttys1));
  newttys1.c_cflag |= (CLOCAL | CREAD);

  switch (nBits) {
    case 7:
      newttys1.c_cflag |= CS7;
      break;
    case 8:
    default:
      newttys1.c_cflag |= CS8;
      break;
  }

  switch (nEvent) {
    case 'O':
      newttys1.c_cflag |= PARENB;
      newttys1.c_iflag |= (INPCK | ISTRIP);
      newttys1.c_cflag |= PARODD;
      break;
    case 'E':
      newttys1.c_cflag |= PARENB;
      newttys1.c_iflag |= (INPCK | ISTRIP);
      newttys1.c_cflag &= ~PARODD;
      break;
    case 'N':
    default:
      newttys1.c_cflag &= ~PARENB;
      break;
  }

  switch (nSpeed) {
    case 2400:
      cfsetispeed(&newttys1, B2400);
      cfsetospeed(&newttys1, B2400);
      break;
    case 4800:
      cfsetispeed(&newttys1, B4800);
      cfsetospeed(&newttys1, B4800);
      break;
    case 9600:
      cfsetispeed(&newttys1, B9600);
      cfsetospeed(&newttys1, B9600);
      break;
    case 230400:
      cfsetispeed(&newttys1, B230400);
      cfsetospeed(&newttys1, B230400);
      break;
    case 460800:
      cfsetispeed(&newttys1, B460800);
      cfsetospeed(&newttys1, B460800);
      break;
    case 115200:
    default:
      cfsetispeed(&newttys1, B115200);
      cfsetospeed(&newttys1, B115200);
      break;
  }

  if (nStop == 1) {
    newttys1.c_cflag &= ~CSTOPB;
  } else if (nStop == 2) {
    newttys1.c_cflag |= CSTOPB;
  }

  newttys1.c_cc[VTIME] = 0;
  newttys1.c_cc[VMIN] = 0;
  tcflush(fd, TCIFLUSH);

  newttys1.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  newttys1.c_oflag &= ~OPOST;
  newttys1.c_iflag &= ~(INLCR | ICRNL | IGNCR);

  if ((tcsetattr(fd, TCSANOW, &newttys1)) != 0) {
    perror("IMU com set error");
    exit(1);
  }

  return 0;
}

void CCodbotBase::DiffCallBack(const geometry_msgs::msg::Twist::SharedPtr cmd_input)
{
  float linear_speed = cmd_input->linear.x;
  float angular_speed = cmd_input->angular.z;

  double linear_sign = 0.0;
  if (linear_speed > 0) {
    linear_sign = 1.0;
  } else if (linear_speed < 0) {
    linear_sign = -1.0;
  }
  linear_speed = std::min(std::abs(linear_speed), myOdomData.cmd_vel_linear_max);
  if (linear_sign != 0.0) {
    linear_speed = std::max(std::abs(linear_speed), myOdomData.cmd_vel_linear_min);
  }
  linear_speed = linear_sign * linear_speed;

  double angular_sign = 0.0;
  if (angular_speed > 0) {
    angular_sign = 1.0;
  } else if (angular_speed < 0) {
    angular_sign = -1.0;
  }
  angular_speed = std::min(std::abs(angular_speed), myOdomData.cmd_vel_anglar_max);
  if (angular_sign != 0.0) {
    angular_speed = std::max(std::abs(angular_speed), myOdomData.cmd_vel_anglar_min);
  }
  angular_speed = angular_sign * angular_speed;

  while (baseCommunication.send_update_flag.load()) {
  }

  baseCommunication.writebuff[0] = 0xCD;
  baseCommunication.writebuff[1] = 0x0A;
  baseCommunication.writebuff[2] = 0x01;
  if (linear_speed >= 0) {
    baseCommunication.writebuff[3] = 0x00;
  } else {
    baseCommunication.writebuff[3] = 0x01;
    linear_speed = -linear_speed;
  }
  baseCommunication.writebuff[4] = (static_cast<int>(linear_speed * 1000) >> 8) & 0xff;
  baseCommunication.writebuff[5] = static_cast<int>(linear_speed * 1000) & 0xff;

  baseCommunication.writebuff[6] = 0x00;
  baseCommunication.writebuff[7] = 0x00;
  baseCommunication.writebuff[8] = 0x00;

  if (angular_speed > 0) {
    baseCommunication.writebuff[9] = 0x00;
  } else {
    baseCommunication.writebuff[9] = 0x01;
    angular_speed *= -1;
  }

  baseCommunication.writebuff[10] = (static_cast<int>(angular_speed * 100) >> 8) & 0xff;
  baseCommunication.writebuff[11] = static_cast<int>(angular_speed * 100) & 0xff;
  baseCommunication.writebuff[12] = 0x00;
  for (int i = 2; i < 12; i++) {
    baseCommunication.writebuff[12] ^= baseCommunication.writebuff[i];
  }

  baseCommunication.send_update_flag.store(true);
}

void CCodbotBase::AckerCallBack(const geometry_msgs::msg::Twist::SharedPtr cmd_input)
{
  float linear_x = cmd_input->linear.x;
  float angle = static_cast<float>(cmd_input->angular.z * 180.0 / M_PI);

  double linear_sign = 0.0;
  if (linear_x > 0) {
    linear_sign = 1.0;
  } else if (linear_x < 0) {
    linear_sign = -1.0;
  }
  linear_x = std::min(std::abs(linear_x), myOdomData.cmd_vel_linear_max);
  if (linear_sign != 0.0) {
    linear_x = std::max(std::abs(linear_x), myOdomData.cmd_vel_linear_min);
  }
  linear_x = linear_sign * linear_x;

  double angle_sign = 0.0;
  if (angle > 0) {
    angle_sign = 1.0;
  } else if (angle < 0) {
    angle_sign = -1.0;
  }
  angle = std::min(std::abs(angle), myOdomData.cmd_vel_angle_max);
  if (angle_sign != 0.0) {
    angle = std::max(std::abs(angle), myOdomData.cmd_vel_angle_min);
  }
  angle = (linear_sign != 0.0) ? static_cast<float>(angle_sign * linear_sign * angle)
                                : static_cast<float>(angle_sign * angle);

  while (baseCommunication.send_update_flag.load()) {
  }

  baseCommunication.writebuff[0] = 0xCD;
  baseCommunication.writebuff[1] = 0x0A;
  baseCommunication.writebuff[2] = 0x01;
  if (linear_x >= 0) {
    baseCommunication.writebuff[3] = 0x00;
  } else {
    baseCommunication.writebuff[3] = 0x01;
    linear_x *= -1;
  }

  baseCommunication.writebuff[4] = (static_cast<int>(linear_x * 1000) >> 8) & 0xff;
  baseCommunication.writebuff[5] = static_cast<int>(linear_x * 1000) & 0xff;
  baseCommunication.writebuff[6] = 0x00;
  baseCommunication.writebuff[7] = 0x00;
  baseCommunication.writebuff[8] = 0x00;

  if (angle >= 0) {
    baseCommunication.writebuff[9] = 0x00;
  } else {
    baseCommunication.writebuff[9] = 0x01;
    angle *= -1;
  }

  baseCommunication.writebuff[10] = (static_cast<int>(angle) >> 8) & 0xff;
  baseCommunication.writebuff[11] = static_cast<int>(angle) & 0xff;
  baseCommunication.writebuff[12] = 0x00;
  for (int i = 2; i < 12; i++) {
    baseCommunication.writebuff[12] ^= baseCommunication.writebuff[i];
  }

  baseCommunication.send_update_flag.store(true);
}

void CCodbotBase::MecanumCallBack(const geometry_msgs::msg::Twist::SharedPtr cmd_input)
{
  float linear_x_speed = cmd_input->linear.x;
  float linear_y_speed = cmd_input->linear.y;
  float angular_speed = cmd_input->angular.z;

  float linear_max_limit = myOdomData.cmd_vel_linear_max;
  float angular_max_limit = myOdomData.cmd_vel_anglar_max;
  linear_x_speed = std::max(-linear_max_limit, std::min(linear_x_speed, linear_max_limit));
  linear_y_speed = std::max(-linear_max_limit, std::min(linear_y_speed, linear_max_limit));
  angular_speed = std::max(-angular_max_limit, std::min(angular_speed, angular_max_limit));

  while (baseCommunication.send_update_flag.load()) {
  }

  baseCommunication.writebuff[0] = 0xCD;
  baseCommunication.writebuff[1] = 0x0A;
  baseCommunication.writebuff[2] = 0x01;
  if (linear_x_speed >= 0) {
    baseCommunication.writebuff[3] = 0x00;
  } else {
    baseCommunication.writebuff[3] = 0x01;
    linear_x_speed = -linear_x_speed;
  }
  baseCommunication.writebuff[4] = (static_cast<int>(linear_x_speed * 1000) >> 8) & 0xff;
  baseCommunication.writebuff[5] = static_cast<int>(linear_x_speed * 1000) & 0xff;

  if (linear_y_speed >= 0) {
    baseCommunication.writebuff[6] = 0x00;
  } else {
    baseCommunication.writebuff[6] = 0x01;
    linear_y_speed = -linear_y_speed;
  }
  baseCommunication.writebuff[7] = (static_cast<int>(linear_y_speed * 1000) >> 8) & 0xff;
  baseCommunication.writebuff[8] = static_cast<int>(linear_y_speed * 1000) & 0xff;

  if (angular_speed > 0) {
    baseCommunication.writebuff[9] = 0x00;
  } else {
    baseCommunication.writebuff[9] = 0x01;
    angular_speed *= -1;
  }

  baseCommunication.writebuff[10] = (static_cast<int>(angular_speed * 100) >> 8) & 0xff;
  baseCommunication.writebuff[11] = static_cast<int>(angular_speed * 100) & 0xff;
  baseCommunication.writebuff[12] = 0x00;
  for (int i = 2; i < 12; i++) {
    baseCommunication.writebuff[12] ^= baseCommunication.writebuff[i];
  }

  baseCommunication.send_update_flag.store(true);
}

void * CCodbotBase::MyReadThread(void * pt)
{
  auto * pt_read = static_cast<CCodbotBase *>(pt);
  unsigned char read_buff = 0;
  double dt = 1.0 / 50.0;
  double previous_time = rclcpp::Clock().now().seconds();
  bool init_yaw_flag = false;

  while (true) {
    int nread = read(pt_read->baseCommunication.serialNum, &read_buff, 1);
    if (nread <= 0) {
      usleep(1000);
      continue;
    }

    pt_read->ReceiveUpdate(13, pt_read->baseCommunication.carVelRaw, read_buff);
    pt_read->ReceiveUpdate(12, pt_read->baseCommunication.motorVelRaw, read_buff);
    pt_read->ReceiveUpdate(5, pt_read->baseCommunication.batteryRaw, read_buff);
    pt_read->ReceiveUpdate(13, pt_read->baseCommunication.imuRpyRaw, read_buff);
    pt_read->ReceiveUpdate(34, pt_read->baseCommunication.imuRaw, read_buff);
    pt_read->ReceiveUpdate(5, pt_read->baseCommunication.carTypeRaw, read_buff);
    pt_read->ReceiveUpdate(6, pt_read->baseCommunication.ultrasonicRaw, read_buff);

    if ((0xfd == pt_read->baseCommunication.carVelRaw[0]) &&
      (0x0a == pt_read->baseCommunication.carVelRaw[1]) &&
      (0x02 == pt_read->baseCommunication.carVelRaw[2]))
    {
      if (pt_read->CheckRawData(pt_read->baseCommunication.carVelRaw, 13, 2)) {
        double current_time = rclcpp::Clock().now().seconds();
        dt = current_time - previous_time;
        previous_time = current_time;
        if (dt <= 0.0) {
          dt = 1.0 / 50.0;
        }

        pt_read->currentBaseData.vx =
          (pt_read->baseCommunication.carVelRaw[3] > 0 ? -1.0F : 1.0F) *
          ((pt_read->baseCommunication.carVelRaw[4] * 256 +
          pt_read->baseCommunication.carVelRaw[5]) / 1000.0F);
        pt_read->currentBaseData.vy =
          (pt_read->baseCommunication.carVelRaw[6] > 0 ? -1.0F : 1.0F) *
          ((pt_read->baseCommunication.carVelRaw[7] * 256 +
          pt_read->baseCommunication.carVelRaw[8]) / 1000.0F);
        pt_read->currentBaseData.angler_speed =
          (pt_read->baseCommunication.carVelRaw[9] > 0 ? -1.0F : 1.0F) *
          ((pt_read->baseCommunication.carVelRaw[10] * 256 +
          pt_read->baseCommunication.carVelRaw[11]) / 100.0F);

        if (std::abs(pt_read->currentBaseData.vx) <= 0.01F) {
          pt_read->currentBaseData.vx = 0.0F;
        }
        if (std::abs(pt_read->currentBaseData.vy) <= 0.01F) {
          pt_read->currentBaseData.vy = 0.0F;
        }
        if (std::abs(pt_read->currentBaseData.angler_speed) <= 0.01F) {
          pt_read->currentBaseData.angler_speed = 0.0F;
        }
        pt_read->baseCommunication.recv_update_flag.store(true);
      }
    }

    if ((0xfd == pt_read->baseCommunication.motorVelRaw[0]) &&
      (0x09 == pt_read->baseCommunication.motorVelRaw[1]) &&
      (0x03 == pt_read->baseCommunication.motorVelRaw[2]))
    {
      if (pt_read->CheckRawData(pt_read->baseCommunication.motorVelRaw, 12, 3)) {
        pt_read->currentBaseData.v1 =
          ((pt_read->baseCommunication.motorVelRaw[3] << 8) +
          pt_read->baseCommunication.motorVelRaw[4]) / 1000.0F;
        pt_read->currentBaseData.v2 =
          ((pt_read->baseCommunication.motorVelRaw[5] << 8) +
          pt_read->baseCommunication.motorVelRaw[6]) / 1000.0F;
        pt_read->currentBaseData.v3 =
          ((pt_read->baseCommunication.motorVelRaw[7] << 8) +
          pt_read->baseCommunication.motorVelRaw[8]) / 1000.0F;
        pt_read->currentBaseData.v4 =
          ((pt_read->baseCommunication.motorVelRaw[9] << 8) +
          pt_read->baseCommunication.motorVelRaw[10]) / 1000.0F;
      }
    }

    if ((0xfd == pt_read->baseCommunication.batteryRaw[0]) &&
      (0x02 == pt_read->baseCommunication.batteryRaw[1]) &&
      (0x04 == pt_read->baseCommunication.batteryRaw[2]))
    {
      if (pt_read->CheckRawData(pt_read->baseCommunication.batteryRaw, 5, 4)) {
        float battery = pt_read->baseCommunication.batteryRaw[3] * 0.1F;
        if (battery < 10.0F) {
          RCLCPP_WARN_THROTTLE(
            pt_read->get_logger(), *pt_read->get_clock(), 5000,
            "Battery is low, please charge: %.1fV", battery);
        }
      }
    }

    if ((0xfd == pt_read->baseCommunication.imuRpyRaw[0]) &&
      (0x0a == pt_read->baseCommunication.imuRpyRaw[1]) &&
      (0x05 == pt_read->baseCommunication.imuRpyRaw[2]))
    {
      if (pt_read->CheckRawData(pt_read->baseCommunication.imuRpyRaw, 13, 5)) {
        pt_read->currentBaseData.yaw =
          ((pt_read->baseCommunication.imuRpyRaw[9] > 0 ? -1.0F : 1.0F) *
          ((pt_read->baseCommunication.imuRpyRaw[10] * 256 +
          pt_read->baseCommunication.imuRpyRaw[11]) / 100.0F)) * M_PI / 180.0F;

        if (pt_read->currentBaseData.yaw < 0) {
          pt_read->currentBaseData.yaw += static_cast<float>(2.0 * M_PI);
        }
        if (!init_yaw_flag) {
          pt_read->currentBaseData.init_yaw = pt_read->currentBaseData.yaw;
          pt_read->currentBaseData.curr_orien = 0.0F;
          init_yaw_flag = true;
        } else {
          pt_read->currentBaseData.curr_orien =
            -(pt_read->currentBaseData.yaw - pt_read->currentBaseData.init_yaw);
          if (pt_read->currentBaseData.curr_orien > M_PI) {
            pt_read->currentBaseData.curr_orien -= static_cast<float>(2.0 * M_PI);
          }
          if (pt_read->currentBaseData.curr_orien < -M_PI) {
            pt_read->currentBaseData.curr_orien += static_cast<float>(2.0 * M_PI);
          }
        }
      }
    }

    if ((0xfd == pt_read->baseCommunication.imuRaw[0]) &&
      (0x1f == pt_read->baseCommunication.imuRaw[1]) &&
      (0x06 == pt_read->baseCommunication.imuRaw[2]))
    {
      if (pt_read->CheckRawData(pt_read->baseCommunication.imuRaw, 34, 6)) {
        sensor_msgs::msg::Imu imu_data;
        imu_data.header.stamp = pt_read->now();
        imu_data.header.frame_id = pt_read->imu_frame;

        imu_data.angular_velocity.x = -1.0 *
          ((pt_read->baseCommunication.imuRaw[3] > 0 ? -1.0 : 1.0) *
          (((pt_read->baseCommunication.imuRaw[4] << 8) +
          pt_read->baseCommunication.imuRaw[5]) / 1000.0));
        imu_data.angular_velocity.y =
          (pt_read->baseCommunication.imuRaw[6] > 0 ? -1.0 : 1.0) *
          (((pt_read->baseCommunication.imuRaw[7] << 8) +
          pt_read->baseCommunication.imuRaw[8]) / 1000.0);
        imu_data.angular_velocity.z =
          (pt_read->baseCommunication.imuRaw[9] > 0 ? -1.0 : 1.0) *
          (((pt_read->baseCommunication.imuRaw[10] << 8) +
          pt_read->baseCommunication.imuRaw[11]) / 1000.0);
        imu_data.angular_velocity_covariance = {1e4, 0, 0, 0, 1e4, 0, 0, 0, 1e4};

        imu_data.linear_acceleration.x = -1.0 *
          ((pt_read->baseCommunication.imuRaw[12] > 0 ? -1.0 : 1.0) *
          (((pt_read->baseCommunication.imuRaw[13] << 8) +
          pt_read->baseCommunication.imuRaw[14]) / 1000.0));
        imu_data.linear_acceleration.y =
          (pt_read->baseCommunication.imuRaw[15] > 0 ? -1.0 : 1.0) *
          (((pt_read->baseCommunication.imuRaw[16] << 8) +
          pt_read->baseCommunication.imuRaw[17]) / 1000.0);
        imu_data.linear_acceleration.z =
          (pt_read->baseCommunication.imuRaw[18] > 0 ? -1.0 : 1.0) *
          (((pt_read->baseCommunication.imuRaw[19] << 8) +
          pt_read->baseCommunication.imuRaw[20]) / 1000.0);
        imu_data.linear_acceleration_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e6};

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, pt_read->currentBaseData.curr_orien);
        imu_data.orientation.w = q.w();
        imu_data.orientation.x = q.x();
        imu_data.orientation.y = q.y();
        imu_data.orientation.z = q.z();
        imu_data.orientation_covariance = {1e-8, 0, 0, 0, 1e-8, 0, 0, 0, 1e-8};

        pt_read->imu_pub->publish(imu_data);
      }
    }

    if ((0xfd == pt_read->baseCommunication.carTypeRaw[0]) &&
      (0x02 == pt_read->baseCommunication.carTypeRaw[1]) &&
      (0x07 == pt_read->baseCommunication.carTypeRaw[2]))
    {
      if (pt_read->CheckRawData(pt_read->baseCommunication.carTypeRaw, 5, 7)) {
        pt_read->feedback_type = std::to_string(pt_read->baseCommunication.carTypeRaw[3]);
      }
    }

    if ((0xfd == pt_read->baseCommunication.ultrasonicRaw[0]) &&
      (0x03 == pt_read->baseCommunication.ultrasonicRaw[1]) &&
      (0x08 == pt_read->baseCommunication.ultrasonicRaw[2]))
    {
      if (pt_read->CheckRawData(pt_read->baseCommunication.ultrasonicRaw, 6, 8)) {
        uint16_t raw_distance =
          static_cast<uint16_t>((pt_read->baseCommunication.ultrasonicRaw[3] << 8) |
          pt_read->baseCommunication.ultrasonicRaw[4]);
        pt_read->PublishUltrasonicRange(raw_distance);
      }
    }

    if (pt_read->baseCommunication.recv_update_flag.load()) {
      pt_read->myOdomData.update_flag = false;
      if (pt_read->myOdomData.oriention > 2 * M_PI) {
        pt_read->myOdomData.oriention -= static_cast<float>(2 * M_PI);
      }
      if (pt_read->myOdomData.oriention < -2 * M_PI) {
        pt_read->myOdomData.oriention += static_cast<float>(2 * M_PI);
      }

      double delta_x = pt_read->coeff_odom_x * dt *
        ((pt_read->currentBaseData.vx * std::cos(pt_read->myOdomData.oriention)) -
        (pt_read->currentBaseData.vy * std::sin(pt_read->myOdomData.oriention)));
      double delta_y = pt_read->coeff_odom_y * dt *
        ((pt_read->currentBaseData.vx * std::sin(pt_read->myOdomData.oriention)) +
        (pt_read->currentBaseData.vy * std::cos(pt_read->myOdomData.oriention)));
      pt_read->myOdomData.position_x += static_cast<float>(delta_x);
      pt_read->myOdomData.position_y += static_cast<float>(delta_y);
      pt_read->myOdomData.twist_x = pt_read->currentBaseData.vx;
      pt_read->myOdomData.twist_y = pt_read->currentBaseData.vy;
      pt_read->myOdomData.twist_th = pt_read->currentBaseData.angler_speed;
      pt_read->myOdomData.oriention = pt_read->currentBaseData.curr_orien;
      pt_read->baseCommunication.recv_update_flag.store(false);
      pt_read->myOdomData.update_flag = true;
    }
  }
}

void CCodbotBase::ReceiveUpdate(int data_len, unsigned char * data_type, unsigned char new_buff)
{
  for (int i = 0; i < data_len - 1; i++) {
    data_type[i] = data_type[i + 1];
  }
  data_type[data_len - 1] = new_buff;
}

bool CCodbotBase::CheckRawData(unsigned char * raw_data, int data_num, int current_code)
{
  unsigned char check_num = 0;
  for (int i = 2; i < (data_num - 1); i++) {
    check_num ^= raw_data[i];
  }

  if ((debug_num >> (current_code - 1)) & 0x01) {
    std::cout << "\033[32m#" << current_code << " ";
    for (int i = 0; i < data_num; ++i) {
      printf("%02x ", raw_data[i]);
    }
    std::cout << "\033[0m" << std::endl;
  }

  if (check_num == raw_data[data_num - 1]) {
    return true;
  }

  RCLCPP_WARN(get_logger(), "frame %d xor check failed", current_code);
  return false;
}

void * CCodbotBase::MyWriteThread(void * th)
{
  auto * pt_write = static_cast<CCodbotBase *>(th);
  while (true) {
    if (pt_write->baseCommunication.send_update_flag.load()) {
      if (pt_write->debug_num & 0x01) {
        std::cout << "\033[32m#1 ";
        for (int i = 0; i < 13; ++i) {
          printf("%02x ", pt_write->baseCommunication.writebuff[i]);
        }
        std::cout << "\033[0m" << std::endl;
      }
      pt_write->baseCommunication.nwrite =
        write(pt_write->baseCommunication.serialNum, pt_write->baseCommunication.writebuff, 13);
      pt_write->baseCommunication.send_update_flag.store(false);
    } else {
      usleep(1000);
    }
  }
}

void CCodbotBase::PublishUltrasonicRange(uint16_t raw_distance)
{
  sensor_msgs::msg::Range range_msg;
  range_msg.header.stamp = now();
  range_msg.header.frame_id = ultrasonic_frame;
  range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  range_msg.field_of_view = ultrasonic_field_of_view_rad_;
  range_msg.min_range = ultrasonic_min_range_m_;
  range_msg.max_range = ultrasonic_max_range_m_;

  double range_m = static_cast<double>(raw_distance) * ultrasonic_unit_scale_m_;
  if (range_m < ultrasonic_min_range_m_) {
    range_m = ultrasonic_min_range_m_;
  }
  if (range_m > ultrasonic_max_range_m_) {
    range_m = ultrasonic_max_range_m_;
  }
  range_msg.range = static_cast<float>(range_m);
  ultrasonic_pub->publish(range_msg);
}

void CCodbotBase::tf_pub_callback()
{
  if (!myOdomData.update_flag) {
    return;
  }

  geometry_msgs::msg::TransformStamped odom_trans;
  geometry_msgs::msg::Quaternion trans_quat;
  tf2::Quaternion odom_quat;
  nav_msgs::msg::Odometry odom;

  odom_quat.setRPY(0.0, 0.0, myOdomData.oriention);
  trans_quat.x = odom_quat.x();
  trans_quat.y = odom_quat.y();
  trans_quat.z = odom_quat.z();
  trans_quat.w = odom_quat.w();

  odom_trans.header.stamp = now();
  odom_trans.header.frame_id = odom_frame;
  odom_trans.child_frame_id = odom_child_frame;
  odom_trans.transform.translation.x = myOdomData.position_x;
  odom_trans.transform.translation.y = myOdomData.position_y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = trans_quat;
  tf_publisher_->sendTransform(odom_trans);

  odom.header.stamp = now();
  odom.header.frame_id = odom_frame;
  odom.child_frame_id = odom_child_frame;
  odom.pose.pose.position.x = myOdomData.position_x;
  odom.pose.pose.position.y = myOdomData.position_y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = trans_quat;
  odom.twist.twist.linear.x = myOdomData.twist_x;
  odom.twist.twist.linear.y = myOdomData.twist_y;
  odom.twist.twist.angular.z = myOdomData.twist_th;
  odom_pub->publish(odom);
}
