#include "../include/my_robot_arm/mobile_base_hardware_usart.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <algorithm>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
class serial_arm:public rclcpp::Node{
public:
    serial_arm():Node("arm_control"){
        my_robot_usart_config arm_config;
        arm_config.derive_id="/dev";
        arm_config.derive_name="ttyUSB1";
        arm_config.baud_rate=9600;
        std::cerr<<"预备处理"<<"\n";

    if (!arm.usart_open(arm_config)) {
        std::cerr << "打开失败\n";
        
    }
    //抓取坐标信息订阅
    arm_sub=this->create_subscription<geometry_msgs::msg::PointStamped>("/arm_target",10,
        std::bind(&serial_arm::target_callback,this,std::placeholders::_1));
    }
private:
//隐藏处理过程
    void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
        std::cerr<<"收到回调";
        //视觉坐标
        float cam_x=msg->point.x;
        float cam_y=msg->point.y;
        float cam_z=msg->point.z;

        //世界坐标变换(yolo节点下发的坐标)
        float target_x=-cam_z;
        float target_y=cam_x;
        float target_z=-cam_y;
        std::cerr << "机械臂目标: x=" << target_x << ", y=" << target_y << ", z=" << target_z << std::endl;

        //计算增量
        float offset_x=target_x-world_x;
        float offset_y=target_y-world_y;
        float offset_z=target_z-world_z;
        std::cerr << "需要移动增量: dx=" << offset_x << ", dy=" << offset_y << ", dz=" << offset_z << std::endl;

        //转成int类型通过串口发给下位机
        int8_t serial_x=std::clamp(static_cast<int8_t>(offset_x*100.0f),(int8_t)-15,(int8_t)15);
        int8_t serial_y=std::clamp(static_cast<int8_t>(offset_y*100.0f),(int8_t)-15,(int8_t)15);
        int8_t serial_z=std::clamp(static_cast<int8_t>(offset_z*100.0f),(int8_t)-15,(int8_t)15);

        std::cerr<<"x="<<int(serial_x)<<"y="<<int(serial_y)<<"z="<<int(serial_z)<<"\n";

        //发送命令
        arm.usart_write(serial_x,serial_y,serial_z);

        world_x+=static_cast<float>(serial_x/100.0f);
        world_y+=static_cast<float>(serial_y/100.0f);
        world_z+=static_cast<float>(serial_z/100.0f);

    }
    float world_x=0.0;
    float world_y=0.0;
    float world_z=0.0;
    my_robot_usart arm;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr arm_sub;

};

int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<serial_arm>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}