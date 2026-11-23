#include "../include/origintank_base_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"  
#include <string>
hardware_interface::CallbackReturn
        origintank::origintankhardware::on_init(const hardware_interface::HardwareInfo & info){
            if(hardware_interface::SystemInterface::on_init(info)
                !=hardware_interface::CallbackReturn::SUCCESS){
                    return hardware_interface::CallbackReturn::ERROR;
                }
            cmd_vel.assign(4,0.0);
            wheel_vel.assign(4,0.0);
            pos_vel.assign(4,0.0);
            wheel_reduis=std::stod(info_.hardware_parameters.at("wheel_reduis"));
            lx=std::stod(info_.hardware_parameters.at("lx"));
            ly=std::stod(info_.hardware_parameters.at("ly"));
            port=info_.hardware_parameters.at("port");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

hardware_interface::CallbackReturn
        origintank::origintankhardware::on_configure(const rclcpp_lifecycle::State & previous_state){   
            (void)previous_state;
            return hardware_interface::CallbackReturn::SUCCESS;
        }

hardware_interface::CallbackReturn
        origintank::origintankhardware::on_activate(const rclcpp_lifecycle::State & previous_state){
            (void)previous_state;
            origintank_interface=std::make_shared<CCodbotBase>();
            if(!origintank_interface->init(port)){
                return hardware_interface::CallbackReturn::ERROR;
            }
            return hardware_interface::CallbackReturn::SUCCESS;
        }

hardware_interface::return_type
        origintank::origintankhardware::read(const rclcpp::Time & time,const rclcpp::Duration & period){
            (void)time;
                const double dt=period.seconds();
                auto vel=origintank_interface->readData();
                double r=wheel_reduis;
                for(size_t i=0;i<4;i++){
                    wheel_vel[i]=vel.vel[i]/r;
                    pos_vel[i]+=wheel_vel[i]*dt;
                }
                return hardware_interface::return_type::OK;
        }

hardware_interface::return_type
        origintank::origintankhardware::write(const rclcpp::Time & time,const rclcpp::Duration & period){
            if(wheel_vel.size()<4){
                return hardware_interface::return_type::OK;
            }
            double r=wheel_reduis;
            double Vx= r*((cmd_vel[0]+cmd_vel[1]+cmd_vel[2]+cmd_vel[3])/4.0);//前后（求解竖直方向线速度）
            double Vy= r*((-cmd_vel[0]+cmd_vel[1]+cmd_vel[2]-cmd_vel[3])/4.0);//左右（求解水平方向线速度）
            double W= r*(-cmd_vel[0]-cmd_vel[1]+cmd_vel[2]+cmd_vel[3])/(4.0*(lx+ly));//旋转角速度,lx+ly为力臂
            origintank_interface->MecanumCallBack(Vx,Vy,W);
            return hardware_interface::return_type::OK;
        }

hardware_interface::CallbackReturn
        origintank::origintankhardware::on_deactivate(const rclcpp_lifecycle::State & previous_state){
            origintank_interface->MecanumCallBack(0,0,0);
            if(origintank_interface->MystopThread()){
                return hardware_interface::CallbackReturn::SUCCESS;
            }
            return hardware_interface::CallbackReturn::ERROR;
        }

std::vector<hardware_interface::StateInterface> origintank::origintankhardware::export_state_interfaces(){
        std::vector<hardware_interface::StateInterface> interfaces;
        for(size_t i=0;i<4;i++){
            interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &wheel_vel[i]
            );
        }
        for(size_t i=0;i<4;i++){
            interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &pos_vel[i]
            );
        }
        return interfaces;
}

std::vector<hardware_interface::CommandInterface> origintank::origintankhardware::export_command_interfaces(){
    std::vector<hardware_interface::CommandInterface> interfaces;
    for(size_t i=0;i<4;i++){
        interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &cmd_vel[i]
        );
    }
    return interfaces;
}

PLUGINLIB_EXPORT_CLASS(origintank::origintankhardware, hardware_interface::SystemInterface)