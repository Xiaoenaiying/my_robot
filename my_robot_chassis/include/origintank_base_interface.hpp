#ifndef origintank_base_interface_HPP 
#define origintank_base_interface_HPP 
#include "hardware_interface/system_interface.hpp"                  // ← 官方基类
#include "../include/my_robot_chassis/origintank_base.h"
#include <vector>
#include <string>

namespace origintank{
  class origintankhardware: public hardware_interface :: SystemInterface{
    public:
    //生命周期节点
      hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
      hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;
      hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    
    //硬件接口
      hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override;
      hardware_interface::return_type
        read(const rclcpp::Time & time,const rclcpp::Duration & period) override;
      hardware_interface::return_type
        write(const rclcpp::Time & time,const rclcpp::Duration & period) override;
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    private:
        std::shared_ptr<CCodbotBase> origintank_interface;
        std::vector<double> cmd_vel;//发给解算模块的四轮速度
        std::vector<double> wheel_vel;//接收到的四轮速度
        std::vector<double> pos_vel;//接收到的弧度值
        std::string port;
        double wheel_reduis;
        double lx;
        double ly;
  };
}


#endif //origintank_base_interface_HPP 