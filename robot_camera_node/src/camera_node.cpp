#include "robot_camera_node/mobile_base_hardware_camera.hpp"
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("camera_node");
    //注册参数，并添加默认值
    node->declare_parameter<std::string>("device_path","/dev/video0");
    node->declare_parameter<int>("width",1280);
    node->declare_parameter<int>("height",720);
    node->declare_parameter<std::string>("format","V4L2_PIX_FMT_MJPEG");
    node->declare_parameter<int>("FPS",30);
    node->declare_parameter<std::string>("flip_mode","HORIZONTAL");

    //得到参数值
    camera_config Camera_conifg;
    node->get_parameter("device_path",Camera_conifg.camera_path);
    node->get_parameter("width",Camera_conifg.camera_width);
    node->get_parameter("height",Camera_conifg.camera_hight);
    node->get_parameter("FPS",Camera_conifg.FPS);

    //设置相机格式
    std::string pixel_formats;
    node->get_parameter("format",pixel_formats);
    if(pixel_formats=="V4L2_PIX_FMT_MJPEG"){
        Camera_conifg.camera_formats=V4L2_PIX_FMT_MJPEG;
    }
    else if(pixel_formats=="V4L2_PIX_FMT_YUYV"){
        Camera_conifg.camera_formats=V4L2_PIX_FMT_YUYV;
    }
    else if(pixel_formats=="V4L2_PIX_FMT_GREY"){
        Camera_conifg.camera_formats=V4L2_PIX_FMT_GREY;
    }
    else if(pixel_formats=="V4L2_PIX_FMT_Y10"){
        Camera_conifg.camera_formats=V4L2_PIX_FMT_Y10;
    }
    else if(pixel_formats=="V4L2_PIX_FMT_Y16"){
        Camera_conifg.camera_formats=V4L2_PIX_FMT_Y16;
    }
    else{
        RCLCPP_WARN(node->get_logger(),"此格式设备不支持该格式，将自动使用V4L2_PIX_FMT_MJPEG格式");
        Camera_conifg.camera_formats=V4L2_PIX_FMT_MJPEG;
    }

    //设置颠倒
    std::string flip_mode;
    node->get_parameter("flip_mode",flip_mode);
    if(flip_mode=="VERTICAL"){
        Camera_conifg.flip_mode=FlipMode::VERTICAL;
    }
    else if(flip_mode=="NONE"){
        Camera_conifg.flip_mode=FlipMode::NONE;
    }
    else if(flip_mode=="BOTH"){
        Camera_conifg.flip_mode=FlipMode::BOTH;
    }
    else if(flip_mode=="HORIZONTAL"){
        Camera_conifg.flip_mode=FlipMode::HORIZONTAL;
    }
    else{
        RCLCPP_ERROR(node->get_logger(),"颠倒错误");
        Camera_conifg.flip_mode=FlipMode::NONE;
    }

    my_robot_camera robot_camera;

    std::string Camera_path="/dev/video";
    std::string device_path;
    //自动寻找可支持设备
    for(int i=0;i<7;++i){
        device_path=Camera_path+std::to_string(i);
        if(robot_camera.camera_open(device_path)){
            robot_camera.camera_formats();
            robot_camera.camera_formats_print();
            Camera_conifg.camera_path=device_path;
            break;
        }
        else{
            RCLCPP_WARN(node->get_logger(),"当前设备不被支持");
            continue;
        }
    }
    if(!robot_camera.camera_formats_setting(Camera_conifg)) 
        return -1;
    if(!robot_camera.camera_init_buf(5))
        return -1;
    if(!robot_camera.camera_stream_on())
        return -1;
    //创建节点发布者
    auto pub=node->create_publisher<sensor_msgs::msg::Image>("Image_raw",3);
    cv::Mat frame;

    rclcpp::Rate loop_rate(Camera_conifg.FPS);

    while(rclcpp::ok()){
        frame=robot_camera.camera_opencv_read(Camera_conifg.camera_width,Camera_conifg.camera_hight,
            Camera_conifg.camera_formats,Camera_conifg.flip_mode);
        if(frame.empty()){
            RCLCPP_WARN_THROTTLE(node->get_logger(),*node->get_clock(),1000,"读取到空帧");
            loop_rate.sleep();
            continue;
        }
        //利用opencv将画面转成ROS2可以发送的格式
        std::string ros2formats;
        if((pixel_formats=="V4L2_PIX_FMT_YUYV")||(pixel_formats=="V4L2_PIX_FMT_MJPEG")){
            ros2formats="bgr8";
        }
        else if((pixel_formats=="V4L2_PIX_FMT_Y16")||(pixel_formats=="V4L2_PIX_FMT_Y10")){
            ros2formats="16UC1";
        }
        else if((pixel_formats=="V4L2_PIX_FMT_GREY")){
            ros2formats="mono8";
        }
        else{
            RCLCPP_WARN_THROTTLE(node->get_logger(),*node->get_clock(),1000,"该格式无法传输");
        }
        auto img_msg=cv_bridge::CvImage(std_msgs::msg::Header(),ros2formats,frame).toImageMsg();
        img_msg->header.stamp=node->now();//打上时间戳
        pub->publish(*img_msg);//发布数据   
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    robot_camera.camera_close();
    rclcpp::shutdown();
}