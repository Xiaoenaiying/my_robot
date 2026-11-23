#include "../include/my_robot_chassis/origintank_base.h"
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <ctime>
#include <iostream>

namespace {auto logger=rclcpp::get_logger("origintank");}

bool CCodbotBase::init(const std::string cmd_port)
{
  SetGorgePro(cmd_port.c_str());
  if (serialNum < 0) {
    return false;
  }
  flag.store(true);
  int th_read_status = pthread_create(&th_base_read, nullptr, MyReadThread, this);
  if (0 != th_read_status) {
    RCLCPP_ERROR(logger, "can't create base read thread: %d", th_read_status);
    flag.store(false);
    close(serialNum);
    serialNum=-1;
    return false;
  }

  int th_write_status = pthread_create(&th_base_write, nullptr, MyWriteThread, this);
  if (0 != th_write_status) {
    RCLCPP_ERROR(logger, "can't create base write thread: %d", th_write_status);
    flag.store(false);
    close(serialNum);
    serialNum=-1;
    return false;
  }
  return true;
}

void CCodbotBase::SetGorgePro(const char * serial_num)
{
  serialNum = open(serial_num, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (-1 == serialNum) {
    RCLCPP_ERROR(logger, "Open %s serial port failed: %s", serial_num, strerror(errno));
    return;
  }

  struct termios my_options;
  if (0 != tcgetattr(serialNum, &my_options)) {
    RCLCPP_ERROR(logger, "Get serial port attribute failed: %s", strerror(errno));
    close(serialNum);
    serialNum = -1;
    return;
  }

  tcflush(serialNum, TCIFLUSH);
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

  if (0 != tcsetattr(serialNum, TCSANOW, &my_options)) {
    RCLCPP_ERROR(logger, "Set %s serial port failed: %s", serial_num, strerror(errno));
    close(serialNum);
    serialNum = -1;
    return;
  }

  RCLCPP_INFO(logger, "Serial port %s opened at 115200", serial_num);
}

// void CCodbotBase::DiffCallBack(double V,double W)
// {
//   float linear_speed = V;
//   float angular_speed = W;

//   double linear_sign = 0.0;
//   if (linear_speed > 0) {
//     linear_sign = 1.0;
//   } else if (linear_speed < 0) {
//     linear_sign = -1.0;
//   }
//   linear_speed = std::min(std::abs(linear_speed),cmd_vel_linear_max);
//   if (linear_sign != 0.0) {
//     linear_speed = std::max(std::abs(linear_speed),cmd_vel_linear_min);
//   }
//   linear_speed = linear_sign * linear_speed;

//   double angular_sign = 0.0;
//   if (angular_speed > 0) {
//     angular_sign = 1.0;
//   } else if (angular_speed < 0) {
//     angular_sign = -1.0;
//   }
//   angular_speed = std::min(std::abs(angular_speed),cmd_vel_anglar_max);
//   if (angular_sign != 0.0) {
//     angular_speed = std::max(std::abs(angular_speed),cmd_vel_anglar_min);
//   }
//   angular_speed = angular_sign * angular_speed;

//   while (baseCommunication.send_update_flag.load()) {
//   }

//   baseCommunication.writebuff[0] = 0xCD;
//   baseCommunication.writebuff[1] = 0x0A;
//   baseCommunication.writebuff[2] = 0x01;
//   if (linear_speed >= 0) {
//     baseCommunication.writebuff[3] = 0x00;
//   } else {
//     baseCommunication.writebuff[3] = 0x01;
//     linear_speed = -linear_speed;
//   }
//   baseCommunication.writebuff[4] = (static_cast<int>(linear_speed * 1000) >> 8) & 0xff;
//   baseCommunication.writebuff[5] = static_cast<int>(linear_speed * 1000) & 0xff;

//   baseCommunication.writebuff[6] = 0x00;
//   baseCommunication.writebuff[7] = 0x00;
//   baseCommunication.writebuff[8] = 0x00;

//   if (angular_speed > 0) {
//     baseCommunication.writebuff[9] = 0x00;
//   } else {
//     baseCommunication.writebuff[9] = 0x01;
//     angular_speed *= -1;
//   }

//   baseCommunication.writebuff[10] = (static_cast<int>(angular_speed * 100) >> 8) & 0xff;
//   baseCommunication.writebuff[11] = static_cast<int>(angular_speed * 100) & 0xff;
//   baseCommunication.writebuff[12] = 0x00;
//   for (int i = 2; i < 12; i++) {
//     baseCommunication.writebuff[12] ^= baseCommunication.writebuff[i];
//   }

//   baseCommunication.send_update_flag.store(true);
// }

// void CCodbotBase::AckerCallBack(double V,double angle_rad)
// {
//   float linear_x = V;
//   float angle = static_cast<float>(angle_rad* 180.0 / M_PI);

//   double linear_sign = 0.0;
//   if (linear_x > 0) {
//     linear_sign = 1.0;
//   } else if (linear_x < 0) {
//     linear_sign = -1.0;
//   }
//   linear_x = std::min(std::abs(linear_x),cmd_vel_linear_max);
//   if (linear_sign != 0.0) {
//     linear_x = std::max(std::abs(linear_x),cmd_vel_linear_min);
//   }
//   linear_x = linear_sign * linear_x;

//   double angle_sign = 0.0;
//   if (angle > 0) {
//     angle_sign = 1.0;
//   } else if (angle < 0) {
//     angle_sign = -1.0;
//   }
//   angle = std::min(std::abs(angle),cmd_vel_angle_max);
//   if (angle_sign != 0.0) {
//     angle = std::max(std::abs(angle),cmd_vel_angle_min);
//   }
//   angle = (linear_sign != 0.0) ? static_cast<float>(angle_sign * linear_sign * angle)
//                                 : static_cast<float>(angle_sign * angle);

//   while (baseCommunication.send_update_flag.load()) {
//   }

//   baseCommunication.writebuff[0] = 0xCD;
//   baseCommunication.writebuff[1] = 0x0A;
//   baseCommunication.writebuff[2] = 0x01;
//   if (linear_x >= 0) {
//     baseCommunication.writebuff[3] = 0x00;
//   } else {
//     baseCommunication.writebuff[3] = 0x01;
//     linear_x *= -1;
//   }

//   baseCommunication.writebuff[4] = (static_cast<int>(linear_x * 1000) >> 8) & 0xff;
//   baseCommunication.writebuff[5] = static_cast<int>(linear_x * 1000) & 0xff;
//   baseCommunication.writebuff[6] = 0x00;
//   baseCommunication.writebuff[7] = 0x00;
//   baseCommunication.writebuff[8] = 0x00;

//   if (angle >= 0) {
//     baseCommunication.writebuff[9] = 0x00;
//   } else {
//     baseCommunication.writebuff[9] = 0x01;
//     angle *= -1;
//   }

//   baseCommunication.writebuff[10] = (static_cast<int>(angle) >> 8) & 0xff;
//   baseCommunication.writebuff[11] = static_cast<int>(angle) & 0xff;
//   baseCommunication.writebuff[12] = 0x00;
//   for (int i = 2; i < 12; i++) {
//     baseCommunication.writebuff[12] ^= baseCommunication.writebuff[i];
//   }

//   baseCommunication.send_update_flag.store(true);
// }

void CCodbotBase::MecanumCallBack(double Vx,double Vy,double W)
{
  float linear_x_speed = Vx;
  float linear_y_speed = Vy;
  float angular_speed = W;

  float linear_max_limit = cmd_vel_linear_max;
  float angular_max_limit =cmd_vel_anglar_max;
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

  while (pt_read->flag.load()) {
    int nread = read(pt_read->serialNum, &read_buff, 1);
    if (nread <= 0) {
      usleep(1000);
      continue;
    }

    pt_read->ReceiveUpdate(13, pt_read->baseCommunication.carVelRaw, read_buff);
    pt_read->ReceiveUpdate(12, pt_read->baseCommunication.motorVelRaw, read_buff);
  
    if ((0xfd == pt_read->baseCommunication.carVelRaw[0]) &&
      (0x0a == pt_read->baseCommunication.carVelRaw[1]) &&
      (0x02 == pt_read->baseCommunication.carVelRaw[2]))
    {
      if (pt_read->CheckRawData(pt_read->baseCommunication.carVelRaw, 13, 2)) {
        std::lock_guard<std::mutex> car_v_wait(pt_read->data_wait);
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
        std::lock_guard<std::mutex> v_wait(pt_read->data_wait);
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
  }
  return nullptr;
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

  RCLCPP_WARN(logger, "frame %d xor check failed", current_code);
  return false;
}

void * CCodbotBase::MyWriteThread(void * th)
{
  auto * pt_write = static_cast<CCodbotBase *>(th);
  while(pt_write->flag.load()){
    if (pt_write->baseCommunication.send_update_flag.load()) {
      if (pt_write->debug_num & 0x01) {
      std::cout << "\033[32m#1 ";
      for (int i = 0; i < 13; ++i) {
        printf("%02x ", pt_write->baseCommunication.writebuff[i]);
      }
        std::cout << "\033[0m" << std::endl;
      }
      write(pt_write->serialNum, pt_write->baseCommunication.writebuff, 13);
      pt_write->baseCommunication.send_update_flag.store(false);
      usleep(10000);
    }
    else{
      usleep(1000);
    }
  }
  return nullptr;
}

bool CCodbotBase::MystopThread(){
  flag.store(false);
  pthread_join(th_base_read,nullptr);
  pthread_join(th_base_write,nullptr);
  close(serialNum);
  serialNum=-1;
  return true;
}

CCodbotBase::database CCodbotBase::readData(){
    database Data_Base;
    std::lock_guard<std::mutex> mutex_lock(data_wait);
    Data_Base.vel[0]=currentBaseData.v1;
    Data_Base.vel[1]=currentBaseData.v2;
    Data_Base.vel[2]=currentBaseData.v3;
    Data_Base.vel[3]=currentBaseData.v4;
    return Data_Base;
}
