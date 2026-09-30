#include "../include/my_robot_usart/mobile_base_hardware_usart.hpp"
#include <chrono>
#include <iostream>
#include <thread>
int main(){
    my_robot_usart_config usart_config;
    usart_config.derive_id="/dev";
    usart_config.derive_name="ttyUSB1";
    usart_config.baud_rate=9600;
    my_robot_usart usart;
    std::cerr<<"预备处理"<<"\n";

    if (!usart.usart_open(usart_config)) {
        std::cerr << "打开失败\n";
        return 1;
    }

    for (int i = 0; i < 600; ++i) {
        if (!/*usart.usart_write(CMD_CHASSIS_CONTROL)*/usart.usart_write(CMD_SERVO_OFFSET_SET)) {  // 内部固定发 0.2 m/s 前进帧
            perror("write");
            return 1;
        }
        usart.usart_write(CMD_SERVOS_RESET);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

    /*std::cerr << "版本查询已发送，开始读取\n";

    my_robot_usart_rx_packet my_robot_rx_packet;
    for(int i=0;i<20;i++){
        if(usart.usart_read(my_robot_rx_packet)){
            if(my_robot_rx_packet.cmd==CMD_VERSION_QUERY){
                std::cout<<"通信成功"<<"\n";
                std::cout<<my_robot_rx_packet.Parsed_data[0]<<"\n";
                std::cout<<my_robot_rx_packet.Parsed_data[1]<<"\n";
                return 0;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
            std::cerr<<"未收到版本回包"<<"\n";*/
}