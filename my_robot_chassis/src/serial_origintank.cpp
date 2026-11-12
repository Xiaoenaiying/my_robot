#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "../include/my_robot_chassis/origintank_base.h"

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<CCodbotBase>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}