#include "../include/robot_camera_node/mobile_base_hardware_camera.hpp"
#include <linux/videodev2.h>
#include <cstring>
#include <opencv2/opencv.hpp>
/*
 *@brief打开相机端口，查看端口支持的类型
 */
bool my_robot_camera::camera_open(std::string device_path){
    std::string Camera_fd=device_path;
    std::string cmera_port=Camera_fd;
    scan_fd=::open(cmera_port.c_str(),O_RDWR);
    if(scan_fd<0){
        std::cerr<<"相机端口不支持打开"<<"\n";
        scan_fd=-1;
        return false;
    }
    v4l2_capability cap;
    memset(&cap,0,sizeof(cap));
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
        std::cerr<<"不支持流式传输"<<"\n";
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
    v4l2_fmtdesc fmtdesc;
    memset(&fmtdesc,0,sizeof(fmtdesc));
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
    v4l2_frmsizeenum frmsize;
    memset(&frmsize,0,sizeof(frmsize));
    v4l2_frmivalenum frmival;
    memset(&frmival,0,sizeof(frmival));
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
    return true;
}
/*
 *@brief先再次初始化，设置采集相机格式，通过ioctl函数（VIDIOC_G_PARM从驱动里获取是否支持，
 VIDIOC_S_PARM设置采集的分辨率和格式
 */
bool my_robot_camera::camera_formats_setting(const camera_config &Camera_config){
    if(scan_fd>0){
        ::close(scan_fd);
        scan_fd=-1;
    }
    v4l2_format fmt{};
    v4l2_streamparm streamparm{};
    std::string camera_real_port=Camera_config.camera_path;
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
    //判断格式是否设置正确
    if(static_cast<__u32>(Camera_config.camera_formats)!=fmt.fmt.pix.pixelformat){
        std::cout<<"注意: 驱动实际设置的格式与请求的格式不一致（可能是驱动不支持）\n";
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
            return false;
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
cv::Mat my_robot_camera::camera_opencv_read(int Camera_width,int Camera_hight,int camera_formats,FlipMode flip_mode){
    if(fd<0){
        std::cout<<"此设备未打开"<<"\n";
        ::close(fd);
        return cv::Mat();
    }
    v4l2_buffer buf;
    buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory=V4L2_MEMORY_MMAP;
    if(0>ioctl(fd,VIDIOC_DQBUF,&buf)){
        std::cout<<"出队失败"<<"\n";
        return cv::Mat();
    }
    camera_address=Camera_buffer[buf.index].start;
    cv::Mat output_frame;
    if(camera_formats==V4L2_PIX_FMT_YUYV){
        cv::Mat camera_yuyv(Camera_hight,Camera_width,CV_8UC2,camera_address);
        cv::cvtColor(camera_yuyv,output_frame,cv::COLOR_YUV2BGR_YUYV);
    }
    //MJPEG压缩数据包装一维数组,再交给imdecode解码显示
    else if(camera_formats==V4L2_PIX_FMT_MJPEG){
        cv::Mat camera_MJPEG(1,static_cast<int>(buf.bytesused),CV_8UC1,camera_address);//把数据包装成一维数组
        output_frame=cv::imdecode(camera_MJPEG,cv::IMREAD_COLOR);
    }
    else if(camera_formats==V4L2_PIX_FMT_GREY){
        cv::Mat camera_GREY(Camera_hight,Camera_width,CV_8UC1,camera_address);
        output_frame=camera_GREY.clone();
    }
    else if(camera_formats==V4L2_PIX_FMT_Y10){
        cv::Mat camera_depth(Camera_hight,Camera_width,CV_16UC1,camera_address);
        output_frame=camera_depth.clone();
    }
    else if(camera_formats==V4L2_PIX_FMT_Y16){
        cv::Mat camera_depth(Camera_hight,Camera_width,CV_16UC1,camera_address);
        output_frame=camera_depth.clone();
    }
    else{
        std::cerr << "错误：不支持的像素格式！\n";
        ioctl(fd,VIDIOC_QBUF,&buf);
        return cv::Mat();
    }
    switch(flip_mode){
        case FlipMode::HORIZONTAL:
            cv::flip(output_frame, output_frame, 1);
            break;
        case FlipMode::BOTH:
            cv::flip(output_frame,output_frame,-1);
            break;
        case FlipMode::NONE:
            break;
        case FlipMode::VERTICAL:
            cv::flip(output_frame,output_frame,0);
            break;
        default:
            break;
    }
    if(0>ioctl(fd,VIDIOC_QBUF,&buf)){
        std::cout<<"入队失败"<<"\n";
        return cv::Mat();
    }
    camera_address=nullptr;
    return output_frame;
}


/*
 *@breif此函数关闭相机描述符，释放资源
 */
void my_robot_camera::camera_close(void){
    if(fd>0){
        v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;//关闭类型为视频采集流
        ioctl(fd,VIDIOC_STREAMOFF,&type);
        //把每块缓冲区的指针交给buf
        for(auto &buf:Camera_buffer){
            //指针不为空（不是 nullptr）而且不是映射失败的返回值
            if(buf.start&&buf.start!=MAP_FAILED){
                munmap(buf.start,buf.length);
            }
        }
        Camera_buffer.clear();//清除缓冲区
        ::close(fd);
        fd=-1;
    }
}