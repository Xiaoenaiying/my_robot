#ifndef mobile_base_hardware_camera_HPP
#define mobile_base_hardware_camera_HPP
#include <fcntl.h>   // open()、O_RDWR、O_NOCTTY、O_SYNC
#include <unistd.h>  // read()、write()、close()
#include <string>
#include "iostream"
#include <sys/ioctl.h>
#include <vector>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <opencv2/opencv.hpp>
  
//opencv里的翻转
enum class FlipMode {
    VERTICAL = 0,   // 对应 cv::flip 的 0(不翻转)
    HORIZONTAL = 1, // 对应 cv::flip 的 1(水平)
    BOTH =2,      // 对应 cv::flip 的 -1(垂直)
    NONE =3        // 注意：OpenCV没有2这个值，所以NONE需要单独处理(180)
};

struct camera_config{
    std::string camera_path;
    int camera_width;
    int camera_hight;
    int camera_formats;//像素格式需要填入和V4L2_PIX_FMT_YUYV类似的参数
    int FPS;//设置帧率
    FlipMode flip_mode;
};

struct camera_buffer{
    void *start=nullptr;
    size_t length=0;
};

class my_robot_camera{
public:
    bool camera_open(std::string device_path);
    void camera_formats(void);
    bool camera_formats_print(void);
    bool camera_formats_setting(const camera_config &Camera_config);
    bool camera_init_buf(int camera_buffer_size);
    bool camera_stream_on(void);
    cv::Mat camera_opencv_read(int Camera_width,int Camera_hight,int camera_formats,FlipMode flip_mode);
    void camera_close(void);
private:
    int fd=-1;
    int scan_fd=-1;//临时扫描
    //保存原始数据
    v4l2_fmtdesc cam_fmts[40]{};
    //保存映射关系
    std::vector<camera_buffer>Camera_buffer;
    //一帧生命周期内保存画面
    void *camera_address;
};










#endif //monile_base_hardware_camera.hpp