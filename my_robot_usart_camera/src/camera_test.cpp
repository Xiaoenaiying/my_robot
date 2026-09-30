#include "../include/mobile_base_hardware_camera.hpp"
int main(){
    camera_config Camera_config;
    my_robot_camera robot_camera;
    Camera_config.camera_id="/dev";
    Camera_config.camera_name="/video0";
    Camera_config.camera_width=1280;
    Camera_config.camera_hight=800;
    Camera_config.camera_formats=V4L2_PIX_FMT_GREY;
    Camera_config.FPS=60;
    for(int i=0;i<7;++i){
        robot_camera.camera_open(i);
        robot_camera.camera_formats();
        robot_camera.camera_formats_print();
    }
    robot_camera.camera_formats_setting(Camera_config);
    robot_camera.camera_init_buf(5);
    robot_camera.camera_stream_on();
    while(1){
        robot_camera.camera_opencv_read(1280,800,V4L2_PIX_FMT_GREY);
    }
    std::cerr << "camera_formats 调用结束\n";
}