#include "../include/mobile_base_hardware_camera.hpp"
#include <opencv2/opencv.hpp>
#include <string>
int main(){
    camera_config Camera_config;
    my_robot_camera robot_camera;
    Camera_config.camera_path="/dev/video0"; 
    Camera_config.camera_width=1280;
    Camera_config.camera_hight=720;
    Camera_config.camera_formats=V4L2_PIX_FMT_MJPEG;
    Camera_config.flip_mode=FlipMode::HORIZONTAL; // OpenCV flip code for vertical flipping
    Camera_config.FPS=30;
    static std::string camera_path;
    //记录有效路径
    static std::string target_camera_path;
    for(int i=0;i<7;++i){
        camera_path="/dev/video"+std::to_string(i);
        if(robot_camera.camera_open(camera_path)){
            robot_camera.camera_formats();
            robot_camera.camera_formats_print();
            target_camera_path=camera_path;
            break;
        }
        else{
            std::cout<<camera_path<<"此设备路径不存在\n";
        }
    }
    if(!robot_camera.camera_formats_setting(Camera_config)) 
        return -1;
    if(!robot_camera.camera_init_buf(5))
        return -1;
    if(!robot_camera.camera_stream_on())
        return -1;
    cv::Mat frame;
    while(1){
        frame=robot_camera.camera_opencv_read(Camera_config.camera_width,Camera_config.camera_hight,
            Camera_config.camera_formats,Camera_config.flip_mode);
        if(!frame.empty()){
            cv::imshow("frame",frame);
        }
        if(cv::waitKey(1)==27)
            break;
    }
    robot_camera.camera_close();
    std::cerr << "camera_formats 调用结束\n";
    return 0;
}