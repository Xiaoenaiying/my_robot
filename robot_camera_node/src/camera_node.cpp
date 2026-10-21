#include "robot_camera_node/mobile_base_hardware_camera.hpp"
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("camera_node");
    //注册参数，并添加默认值
    node->declare_parameter<std::string>("device_path","/dev/video4");
    node->declare_parameter<int>("width",640);
    node->declare_parameter<int>("height",400);
    node->declare_parameter<std::string>("format","V4L2_PIX_FMT_MJPEG");
    node->declare_parameter<int>("FPS",30);
    node->declare_parameter<std::string>("flip_mode","HORIZONTAL");

    //得到参数值
    camera_config Camera_conifg;
    node->get_parameter("device_path",Camera_conifg.camera_path);
    node->get_parameter("width",Camera_conifg.camera_width);
    node->get_parameter("height",Camera_conifg.camera_hight);
    node->get_parameter("FPS",Camera_conifg.FPS);

    //深度相机
    node->declare_parameter<std::string>("depth_path","/dev/video0");
    node->declare_parameter<int>("depth_width",640);
    node->declare_parameter<int>("depth_height",400);
    node->declare_parameter<std::string>("depth_format","V4L2_PIX_FMT_Y16");
    node->declare_parameter<int>("depth_FPS",30);
    node->declare_parameter<std::string>("depth_flip_mode","HORIZONTAL");

    //得到参数值
    camera_config depth_conifg;
    node->get_parameter("depth_path",depth_conifg.camera_path);
    node->get_parameter("depth_width",depth_conifg.camera_width);
    node->get_parameter("depth_height",depth_conifg.camera_hight);
    node->get_parameter("depth_FPS",depth_conifg.FPS);

    //设置相机格式
    std::string pixel_formats;
    std::string depth_formats;
    node->get_parameter("format",pixel_formats);
    node->get_parameter("depth_format",depth_formats);
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

    if(depth_formats=="V4L2_PIX_FMT_Y10"){
        depth_conifg.camera_formats=V4L2_PIX_FMT_Y10;
    }
    else if(depth_formats=="V4L2_PIX_FMT_Y16"){
        depth_conifg.camera_formats=V4L2_PIX_FMT_Y16;
    }
    else{
        depth_conifg.camera_formats=V4L2_PIX_FMT_Y16;
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

    my_robot_camera robot_camera;//RGB相机实例
    my_robot_camera robot_depth;//深度相机实例

    if(!robot_camera.camera_formats_setting(Camera_conifg)||!robot_depth.camera_formats_setting(depth_conifg)) 
        return -1;
    if(!robot_camera.camera_init_buf(5)||!robot_depth.camera_init_buf(5))
        return -1;
    if(!robot_camera.camera_stream_on()||!robot_depth.camera_stream_on())
        return -1;
    //创建RGB相机节点发布者
    auto color_pub=node->create_publisher<sensor_msgs::msg::Image>("Image_raw",3);
    //创建深度相机节点发布者
    auto depth_pub=node->create_publisher<sensor_msgs::msg::Image>("depth_raw",3);

    rclcpp::Rate loop_rate(Camera_conifg.FPS);
    rclcpp::Rate depth_loop_rate(depth_conifg.FPS);

    //RGB相机线程
    std::thread colcor_camera([&](){
    while(rclcpp::ok()){
        cv::Mat frame;
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
            color_pub->publish(*img_msg);//发布数据   
            loop_rate.sleep();
        }
    });

    //深度相机线程
    std::thread depth_camera([&](){
    while(rclcpp::ok()){
    cv::Mat frame;
    frame=robot_depth.camera_opencv_read(depth_conifg.camera_width,depth_conifg.camera_hight,
            depth_conifg.camera_formats,depth_conifg.flip_mode);
        if(frame.empty()){
            RCLCPP_WARN_THROTTLE(node->get_logger(),*node->get_clock(),1000,"读取到空帧");
            loop_rate.sleep();
            continue;
        }
        //利用opencv将画面转成ROS2可以发送的格式
        std::string ros2formats;
            ros2formats="16UC1";
            auto img_msg=cv_bridge::CvImage(std_msgs::msg::Header(),ros2formats,frame).toImageMsg();
            img_msg->header.stamp=node->now();//打上时间戳
            depth_pub->publish(*img_msg);//发布数据   
            loop_rate.sleep();
        }
    });
    std::this_thread::sleep_for(std::chrono::seconds(1));
    rclcpp::spin(node);
    colcor_camera.join();
    robot_camera.camera_close();
    depth_camera.join();
    robot_depth.camera_close();
    rclcpp::shutdown();
}
