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
#include "rclcpp/rclcpp.hpp"
#include <atomic>
#include <vector>

#define origintank_diff 0 //差速
#define origintank_mecanum 1 //麦轮
#define origintank_Acker 2 //阿克曼轮

#ifndef origintank_type
#define origintank_type origintank_mecanum //默认麦轮
#endif

class CCodbotBase
{
public:
  bool init(const std::string cmd_port);
  void SetGorgePro(const char * serial_num);
  #if origintank_type==origintank_diff
    void DiffCallBack(double V,double W);
  #elif origintank_type==origintank_Acker
    void AckerCallBack(double V,double angle_rad);
  #elif origintank_type==origintank_mecanum
    void MecanumCallBack(double Vx,double Vy,double W);
  #else
    #error "origintank type delete faile"
  #endif

  static void * MyReadThread(void * pt);
  static void * MyWriteThread(void * th);
  bool MystopThread();
  void ReceiveUpdate(int data_len, unsigned char * data_type, unsigned char new_buff);
  bool CheckRawData(unsigned char * raw_data, int data_num, int current_code);
  struct database
  {
    double vel[4];
  };
  database readData();

private:
  pthread_t th_base_write {};
  pthread_t th_base_read {};
  int debug_num=0;

  //线速度
  static constexpr double cmd_vel_linear_max=0.3;

  static constexpr double cmd_vel_linear_min=0.0;

  //角速度限幅
  static constexpr double cmd_vel_anglar_max=2.0;

  static constexpr double cmd_vel_anglar_min=0.0;
 
  //转向角限幅
  static constexpr double cmd_vel_angle_max=25.0;
 
  static constexpr double cmd_vel_angle_min=0.0;

  std::mutex data_wait;

  int serialNum {-1};
  
  std::atomic_bool flag {false};//记录原子变量标志位，用于多线程

  struct BaseReader
  {
    unsigned char carVelRaw[13] = {0};
    unsigned char motorVelRaw[12] = {0};
    unsigned char writebuff[13] = {0xcd, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
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

};


#endif  // ORIGINTANK_BASE__ORIGINTANK_BASE_H_
