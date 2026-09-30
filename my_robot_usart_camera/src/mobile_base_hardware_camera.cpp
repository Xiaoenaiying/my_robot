#include "../include/mobile_base_hardware_camera.hpp"
#include <linux/videodev2.h>
#include <cstring>
#include <opencv2/opencv.hpp>
/*
 *@brief打开相机端口，查看端口支持的类型
 */
bool my_robot_camera::camera_open(int n){
    std::string Camera_id="/dev";
    std::string Camera_name="video";
    Camera_name+=std::to_string(n);
    std::string Camera_fd=Camera_id+"/"+Camera_name;
    std::string cmera_port=Camera_fd;
    scan_fd=::open(cmera_port.c_str(),O_RDWR);
    if(scan_fd<0){
        std::cerr<<"相机端口不支持打开"<<"\n";
        scan_fd=-1;
        return false;
    }
    v4l2_capability cap={0};
    if(::ioctl(scan_fd,VIDIOC_QUERYCAP,&cap)<0){
        scan_fd=-1;
        return false;
    }
    if(!(V4L2_CAP_VIDEO_CAPTURE&cap.capabilities)){
        std::cerr<<"不支持视频采集"<<"\n";
        scan_fd=-1;
        return false;
    }
    if(!(V4L2_CAP_STREAMING&cap.capabilities)){
        std::cerr<<"不支持流式传输"<<"n";
    }
    std::cout<<"成功打开"<<Camera_fd<<"设备";
    return true;
}

/*
 *@brief保存格式，分辨率
 */
void my_robot_camera::camera_formats(void){
    memset(cam_fmts,0,sizeof(cam_fmts));
    //原始数据
    v4l2_fmtdesc fmtdesc={0};
    fmtdesc.index=0;
    fmtdesc.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    while(0==ioctl(scan_fd,VIDIOC_ENUM_FMT,&fmtdesc)){
        cam_fmts[fmtdesc.index].pixelformat=fmtdesc.pixelformat;
        std::strncpy(reinterpret_cast<char*>(cam_fmts[fmtdesc.index].description),
                     reinterpret_cast<const char*>(fmtdesc.description),
                     sizeof(cam_fmts[fmtdesc.index].description) - 1);
        cam_fmts[fmtdesc.index].description[sizeof(cam_fmts[fmtdesc.index].description) - 1] = '\0';
        ++fmtdesc.index;
    }
}

/*
 *@brief打印格式
 */
bool my_robot_camera::camera_formats_print(void){
    v4l2_frmsizeenum frmsize{0};
    v4l2_frmivalenum frmival{0};
    frmsize.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    frmival.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for(int i=0;cam_fmts[i].pixelformat;++i){
        //打印摄像头支持的像素格式
        std::cout<<"format<0x"<<std::hex<<cam_fmts[i].pixelformat<<std::dec<<
        ">,description<"<<cam_fmts[i].description<<">"<<"\n";
        //打印摄像头支持的分辨率
        frmsize.index=0;
        frmsize.pixel_format=cam_fmts[i].pixelformat;
        frmival.pixel_format=cam_fmts[i].pixelformat;
        while(0==ioctl(scan_fd,VIDIOC_ENUM_FRAMESIZES,&frmsize)){
            std::cout<<"size<"<<frmsize.discrete.width<<"*"<<frmsize.discrete.height<<">";
            frmsize.index++;
            //打印帧率
            frmival.index=0;
            frmival.width=frmsize.discrete.width;
            frmival.height=frmsize.discrete.height;
            while(0==ioctl(scan_fd,VIDIOC_ENUM_FRAMEINTERVALS,&frmival)){
                std::cout<<"<"<<frmival.discrete.denominator/frmival.discrete.numerator<<"FPS>";
                ++frmival.index;
            }
            std::cout<<"\n";
        }
        std::cout<<"\n";
    }
    return false;
}
/*
 *@brief先再次初始化，设置采集相机格式，通过ioctl函数（VIDIOC_G_PARM从驱动里获取是否支持，
 VIDIOC_S_PARM设置采集的分辨率和格式
 */
bool my_robot_camera::camera_formats_setting(const camera_config &Camera_config){
    v4l2_format fmt{};
    v4l2_streamparm streamparm{};
    std::string camera_real_port=Camera_config.camera_id+"/"+Camera_config.camera_name;
    video_dervice=Camera_config.camera_name;
    fd=::open(camera_real_port.c_str(),O_RDWR);
    if(0>fd){
        std::cerr<<"打开目标相机失败"<<"\n";
        return false;
    }
    fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;//采集格式
    fmt.fmt.pix.width=Camera_config.camera_width;
    fmt.fmt.pix.height=Camera_config.camera_hight;
    //YUYV格式
    if(Camera_config.camera_formats==V4L2_PIX_FMT_YUYV){
        fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_YUYV;//像素格式
    }
    //motion-jepg格式
    if(Camera_config.camera_formats==V4L2_PIX_FMT_MJPEG){
        fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_MJPEG;
    }
    //GREY
    if(Camera_config.camera_formats==V4L2_PIX_FMT_GREY){
        fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_GREY;
    }
    //10  Greyscale
    if(Camera_config.camera_formats==V4L2_PIX_FMT_Y10){
        fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_Y10;
    }
    //16  Greyscale
    if(Camera_config.camera_formats==V4L2_PIX_FMT_Y16){
        fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_Y16;
    }
    if(0>ioctl(fd,VIDIOC_S_FMT,&fmt)){
        std::cout<<"ioctl:error"<<"\n";
        return false;
    }
    //判断是否设置为YUYV像素格式
    if(!(V4L2_PIX_FMT_YUYV==fmt.fmt.pix.pixelformat)){
        std::cout<<"Error:the drivce does not support YUYV format\n";
        return false;
    }
    if(!(V4L2_PIX_FMT_MJPEG==fmt.fmt.pix.pixelformat)){
        std::cout<<"Error:the drivce does not support MJPEG format\n";
        return false;
    }
    if(!(V4L2_PIX_FMT_GREY==fmt.fmt.pix.pixelformat)){
        std::cout<<"Error:the drivce does not support GREY format\n";
        return false;
    }
    if(!(V4L2_PIX_FMT_Y10==fmt.fmt.pix.pixelformat)){
        std::cout<<"Error:the drivce does not support Y10 format\n";
        return false;
    }
    if(!(V4L2_PIX_FMT_Y16==fmt.fmt.pix.pixelformat)){
        std::cout<<"Error:the drivce does not support Y16 format\n";
        return false;
    }
    int frm_width=fmt.fmt.pix.width;//获取实际的帧宽度
    int frm_hight=fmt.fmt.pix.height;//获取实际的帧高度
    //打印实际的分辨率
    std::cout<<"size<"<<frm_width<<"*"<<frm_hight<<">"<<"\n";
    //设置帧率
    streamparm.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd,VIDIOC_G_PARM,&streamparm);
    if(V4L2_CAP_TIMEPERFRAME&streamparm.parm.capture.capability){
        streamparm.parm.capture.timeperframe.numerator=1;//表示时间（通常为1）
        streamparm.parm.capture.timeperframe.denominator=Camera_config.FPS;//表示多少帧
        if(0>ioctl(fd,VIDIOC_S_PARM,&streamparm)){
            std::cout<<"error:帧率设置失败"<<"\n";
            return false;
        }
    }
    return true;
}

/*
 *@brief初始化缓冲区
 *    PROT_READ | PROT_WRITE,  // 允许读写
 * MAP_SHARED与驱动共享缓冲区
 */
bool my_robot_camera::camera_init_buf(int camera_buffer_size){
    v4l2_requestbuffers reqbuf;
    v4l2_buffer buf;
    reqbuf.count=camera_buffer_size;
    reqbuf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory=V4L2_MEMORY_MMAP;//内核自动分配，映射到用户
    if(0>ioctl(fd,VIDIOC_REQBUFS,&reqbuf)){
        std::cout<<"申请缓冲区失败"<<"\n";
        return false;
    }
    Camera_buffer.resize(reqbuf.count);
    /*建立内存映射*/
    buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory=V4L2_MEMORY_MMAP;
    //让四个数组内部指针和mmap指针建立内存映射的关系
    for(buf.index=0;buf.index<reqbuf.count;++buf.index){
        ioctl(fd,VIDIOC_QUERYBUF,&buf);
        Camera_buffer[buf.index].length=buf.length;
        Camera_buffer[buf.index].start=mmap(nullptr,buf.length,PROT_READ | PROT_WRITE,MAP_SHARED,
        fd,buf.m.offset);
        if(MAP_FAILED==Camera_buffer[buf.index].start){
            std::cerr<<"内存映射失败"<<"\n";
        }
    }
    for(buf.index=0;buf.index<reqbuf.count;++buf.index){
        if(0>ioctl(fd,VIDIOC_QBUF,&buf)){
        std::cerr<<"入队失败"<<"\n";
        return false;
        }
        else{
            std::cerr<<"处理完第"<<buf.index<<"缓冲区"<<"\n";
        }
    }
    return true;
}

bool my_robot_camera::camera_stream_on(void){
    v4l2_buf_type type;
    type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(0>ioctl(fd,VIDIOC_STREAMON,&type)){
        std::cerr<<"打开视频采集流失败"<<"\n";
        return false;
    }
    std::cout<<"打开视频采集流没有问题"<<"\n";
    return true;
}

/*
 *@brief调用opencv接收图像显示 ，开始真的出队（处理信息），入队（保存摄像头画面缓冲区指针）
 */
bool my_robot_camera::camera_opencv_read(int Camera_width,int Camera_hight,int camera_formats){
    v4l2_buffer buf;
    buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory=V4L2_MEMORY_MMAP;
    if(0>ioctl(fd,VIDIOC_DQBUF,&buf)){
        std::cout<<"出队失败"<<"\n";
        return false;
    }
    camera_address=Camera_buffer[buf.index].start;
    if(camera_formats==V4L2_PIX_FMT_YUYV){
        cv::Mat camera_yuyv(Camera_hight,Camera_width,CV_8UC2,camera_address);
        cv::Mat camera_bgr;
        cv::cvtColor(camera_yuyv,camera_bgr,cv::COLOR_YUV2BGR_YUYV);
        //颠倒画面
        cv::flip(camera_bgr,camera_bgr,-1);
        cv::imshow("camera_bgr",camera_bgr);
    }
    //MJPEG压缩数据包装一维数组,再交给imdecode解码显示
    if(camera_formats==V4L2_PIX_FMT_MJPEG){
        cv::Mat camera_MJPEG(1,static_cast<int>(buf.bytesused),CV_8UC1,camera_address);//把数据包装成一维数组
        cv::Mat camera_rgb=cv::imdecode(camera_MJPEG,cv::IMREAD_COLOR);
        if(video_dervice=="/video6"){
            cv::flip(camera_rgb,camera_rgb,-1);
        }
        cv::imshow("camera_rgb",camera_rgb);
    }
    if(camera_formats==V4L2_PIX_FMT_GREY){
        cv::Mat camera_GREY(Camera_hight,Camera_width,CV_8UC1,camera_address);
        cv::Mat grey_video;
        //调用cv::GaussianBlur来实现高斯模糊函数，使得图像光滑,主要是第三个参数高斯核
        cv::GaussianBlur(camera_GREY,grey_video,cv::Size(5,5),0);
        cv::imshow("grey_video",grey_video);
    }
    if(0>ioctl(fd,VIDIOC_QBUF,&buf)){
        std::cout<<"入队失败"<<"\n";
        return false;
    }
    cv::waitKey(1);
    camera_address=nullptr;
    return true;
}