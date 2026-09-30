#ifndef mobile_base_hardware_usart_HPP
#define mobile_base_hardware_usart_HPP

#include <fcntl.h>   // open()、O_RDWR、O_NOCTTY、O_SYNC
#include <unistd.h>  // read()、write()、close()
#include <termios.h>  // 波特率相关
#include <string>
#include <vector>
#include <sys/ioctl.h>
#include <thread>
#include <chrono>

#define APP_PACKET_HEADER                  0x55  /* 通信协议帧头 */

typedef enum
{
	CMD_VERSION_QUERY = 1,
	CMD_SERVO_OFFSET_READ,
	CMD_MULT_SERVO_MOVE,
	CMD_COORDINATE_SET = 4,
	CMD_ACTION_GROUP_RUN = 6,
	CMD_FULL_ACTION_STOP,
	CMD_FULL_ACTION_ERASE,
	CMD_CHASSIS_CONTROL,
	CMD_SERVO_OFFSET_SET,	
	CMD_SERVO_OFFSET_DOWNLOAD,
	CMD_SERVOS_RESET,
	CMD_ANGLE_BACK_READING,
	CMD_ACTION_DOWNLOAD = 25,
	CMD_FUNC_NULL
}AppFunctionStatus;

struct my_robot_usart_config
{
    std::string derive_id;
	std::string derive_name;
	int baud_rate=9600;
};

struct my_robot_usart_rx_packet{
	int cmd=0;
	std::vector<int> rx_packet;
	std::vector<int> Parsed_data;
};

class my_robot_usart{
public:
    bool usart_open(const my_robot_usart_config &usart_config);
    bool usart_read(my_robot_usart_rx_packet &my_robot_rx_packet);
    bool usart_write(AppFunctionStatus appsatus);
	bool unpack(my_robot_usart_rx_packet &my_robot_rx_packet);
private:
    int fd_=-1;
    my_robot_usart_config usart_conifg;
	speed_t to_linux_baud(int Baud_rate);
};
#endif //mobile_base_hardware_usart_HPP