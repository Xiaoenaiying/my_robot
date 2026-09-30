#include "../include/my_robot_usart/mobile_base_hardware_usart.hpp"
#include <iostream>
/*
 *brief打开串口
 *参数：usart_config传入与串口通信相关信息
 */
/*bool my_robot_usart::usart_open(const my_robot_usart_config &usart_config){
    const std::string port=usart_config.derive_id+"/"+usart_config.derive_name;
    fd_=::open(port.c_str(),O_RDWR | O_NOCTTY | O_SYNC);
    termios tty{};
    speed_t baud_rate=to_linux_baud(usart_config.baud_rate);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return false;
    }
    cfsetispeed(&tty,baud_rate);
    cfsetospeed(&tty, baud_rate);
    tty.c_cc[VMIN] = 0;   // 没有数据时不强制等满指定字节
    tty.c_cc[VTIME] = 1;  // 最多等 0.1 秒
    tcsetattr(fd_, TCSANOW, &tty);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return false;
    }
    return fd_>=0;
}*/

bool my_robot_usart::usart_open(
    const my_robot_usart_config &usart_config)
{
    const std::string port =
        usart_config.derive_id + "/" + usart_config.derive_name;

    std::cerr<< "打开: " << port << "\n";

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        perror("open");
        return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        perror("tcgetattr");
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    const speed_t baud_rate = to_linux_baud(usart_config.baud_rate);
    if (baud_rate == 0) {
        std::cerr << "不支持的波特率\n";
        return false;
    }

    cfsetispeed(&tty, baud_rate);
    cfsetospeed(&tty, baud_rate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8 位数据
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS); // 无校验、1停止位、无流控

    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;  // 原始模式，VMIN/VTIME 才按预期生效

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;  // read 最多等 0.1 秒

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    int modem_bits = TIOCM_DTR | TIOCM_RTS;
    if (::ioctl(fd_, TIOCMBIC, &modem_bits) != 0) {
        perror("关闭 DTR/RTS");
    // 先不退出，部分 USB 转串口不支持该设置
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));

    if (tcflush(fd_, TCIOFLUSH) != 0) {
        perror("tcflush");
        return false;
}

    return true;
}

/*
 *brief读取数据
 *参数：my_robot_rx_packet用于接收缓冲区
 */
bool my_robot_usart::usart_read(my_robot_usart_rx_packet &my_robot_rx_packet){
    uint8_t temp[20];
    ssize_t count=::read(fd_,temp,sizeof(temp));
    if(count>0){
        std::cerr << "RX " << count << " bytes: ";
        for (ssize_t i = 0; i < count; ++i) {
            std::cerr << std::hex
              << static_cast<int>(temp[i]) << ' ';
        }
        std::cerr << std::dec << '\n';
        my_robot_rx_packet.rx_packet.insert(my_robot_rx_packet.rx_packet.end(),
        temp,temp+count);
        return unpack(my_robot_rx_packet);
    }
    else{
        return false;
    }
    
}

/*
 *brief发送数据
 *参数appsatus与控制逻辑相关
 *内容：通过将字节打包成vector进行发送
 */
bool my_robot_usart::usart_write(AppFunctionStatus appsatus){
    /*const std::vector<uint8_t> tx = {
        0xCD, 0x0A, 0x01,
        0x00, 0x00, 0xC8,  // vx = +50，0.05 × 1000
        0x00, 0x00, 0x00,  // vy = 0
        0x01, 0x00, 0x00,  // wz = 0；按旧代码零值走此符号分支
        0xC8               // XOR 校验
    };*/
    std::vector<uint8_t>tx;
    tx.push_back(APP_PACKET_HEADER);
    tx.push_back(APP_PACKET_HEADER);
    tx.push_back(8);
    tx.push_back(static_cast<uint8_t>(appsatus));
    tx.push_back(6);          // 舵机数
    tx.push_back(0xE8);       // 1000 ms 低字节
    tx.push_back(0x03);       // 1000 ms 高字节
    tx.push_back(5);          // ID 1
    tx.push_back(0xC8);       // 脉宽 450 低字节
    tx.push_back(0x01);       // 脉宽 450 高字节
    return ::write(fd_,tx.data(),tx.size())==static_cast<ssize_t>(tx.size());
}

speed_t my_robot_usart::to_linux_baud(int Baud_rate){
    switch(Baud_rate)
    {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default: return 0;
    }
}

/*
 *brief解析接收数据
 *参数：my_robot_rx_packet中保存解析后的数据
 *内容：先判断对方主机能否通信，然后读取命令和参数字长，然后把第四个到最后一个读进保存区。如果断字节，则重新接收。
 */
bool my_robot_usart::unpack(my_robot_usart_rx_packet &my_robot_rx_packet){
    while(my_robot_rx_packet.rx_packet.size()>=4){
        if((my_robot_rx_packet.rx_packet[0]!=APP_PACKET_HEADER)||
            (my_robot_rx_packet.rx_packet[1])!=APP_PACKET_HEADER)
            {
                my_robot_rx_packet.rx_packet.erase(my_robot_rx_packet.rx_packet.begin());
                continue;
            }
        else{
                int length=my_robot_rx_packet.rx_packet[2];
                std::size_t Reallength=static_cast<std::size_t>(2 + length);
                if(length<2){
                    my_robot_rx_packet.rx_packet.erase(my_robot_rx_packet.rx_packet.begin());
                    continue;
                }
                if(my_robot_rx_packet.rx_packet.size()<Reallength){
                    return false;
                }
                my_robot_rx_packet.cmd=static_cast<AppFunctionStatus>(my_robot_rx_packet.rx_packet[3]);
                my_robot_rx_packet.Parsed_data.assign(my_robot_rx_packet.rx_packet.begin()+4,
                    my_robot_rx_packet.rx_packet.begin()+Reallength);
                my_robot_rx_packet.rx_packet.erase(my_robot_rx_packet.rx_packet.begin(),
                    my_robot_rx_packet.rx_packet.begin()+Reallength);
                return true;
            }
    }
    return false;
}


