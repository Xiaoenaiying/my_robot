#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ShapeRecognizer(Node):
    def __init__(self):
        '''调用父类构造函数，顺便初始化父类'''
        super().__init__("shape_recognizer_node")

        #------设置相关参数--------------------------------
        self.declare_parameter("min_area",500)#最小面积阈值（过滤噪点）
        self.declare_parameter("epsilon_ratio",0.02)# 多边形逼近精度
        self.declare_parameter("blur_kernel",5)#高斯核去噪点

        #给类追加成员
        self.min_area=self.get_parameter("min_area").value
        self.epsilon_ratio=self.get_parameter("epsilon_ratio").value
        self.blur_kernel=self.get_parameter("blur_kernel").value

        #订阅摄像头发布者节点
        self.sub=self.create_subscription(Image,"/Image_raw",self.image_callback,3)
        #发布接收到的图像
        self.pub_debug=self.create_publisher(Image,"/shape/debug_image",3)
        #Canny处理后图像
        self.pubCanny=self.create_publisher(Image,"shape/Canny",3)
        #形态学操作后的图像
        self.pub_edge=self.create_publisher(Image,"shape/edge",3)
        #识别后图像
        self.debug=self.create_publisher(Image,"shape/pub_debug",3)
        #实例化cv_bridge用于ROS与opencv格式转换
        self.bridge=CvBridge()
        self.get_logger().info("图像识别节点已启动")

    #--------------------图像回调的实现---------------------------
    def image_callback(self,msg):
        try:
            frame=self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().info(f"转换失败：{e}")
            return 
        
        shapes=self.detect_shapes(frame)

        for s in shapes:
            self.get_logger().info(f"类型为{s['type']},中心点为{s['cx']}{s['cy']}"
                                    f"面积为{s['area']},顶点数{s['vertex_count']}")
        cam_debug=self.draw_result(frame,shapes)
        self.debug.publish(self.bridge.cv2_to_imgmsg(cam_debug,"mono8"))



    #--------图像处理---------------------------------------------
    def detect_shapes(self,frame):
        results=[]
        #将彩色图变为灰度图
        GRAY=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        #高斯模糊（对细小的噪点降噪）高斯核大小（5，5）
        imgBlur=cv2.GaussianBlur(GRAY,(5,5),0)
        #Canny边缘检测
        Canny=cv2.Canny(imgBlur,50,150)
        #形态学操作（腐蚀再膨胀）闭运算
        Kernel=np.ones((3,3),np.uint8)
        Edge=cv2.morphologyEx(Canny,cv2.MORPH_CLOSE,Kernel)
        #self.pubCanny.publish(self.bridge.cv2_to_imgmsg(Canny,"mono8"))
        #self.pub_edge.publish(self.bridge.cv2_to_imgmsg(Edge,"mono8"))
        #轮廓检测
        contours,_=cv2.findContours(Edge,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        #1.面积过滤
        for cnt in contours:
            area=cv2.contourArea(cnt)
            if area<self.min_area:
                continue

        #2.算周长得到长度，用于后面逼近
            perimeter=cv2.arcLength(cnt,True)

        #3.多边形逼近，得到顶点数
            epsilon=perimeter*self.epsilon_ratio#多边形逼近过程中所能允许的最大偏差
            approx=cv2.approxPolyDP(cnt,epsilon,True)#多边形顶点列表
            vertex_count=len(approx)#获取顶点数
            type=self.classify_shape(approx,vertex_count,area,perimeter)

        #4.计算形状中心
            M=cv2.moments(cnt)
            if(M["m00"]==0):
                continue
            cx=M["m10"]/M["m00"]
            cy=M["m01"]/M["m00"]

            results.append({
                "type":type,
                "cx":cx,
                "cy":cy,
                "area":area,
                "vertex_count":vertex_count,
                "approx":approx
            })
        return results

    #---------计算多边形的形状-----------------------------------------------
    def classify_shape(self,approx,vertex_count,area,perimeter):
        if(vertex_count==3):
            return "triangle"
        elif(vertex_count==4):
            x,y,w,h=cv2.boundingRact(approx)
            aspect=w/float(h)#计算宽长比
            if (0.9<=aspect) and (aspect<=1.1):
                return "square"
            else:
                return "rectangle"
        elif(vertex_count==5):
            return "Pentagram"
        elif(vertex_count==6):
            circularity=4*np.pi*area/(perimeter*perimeter)#计算圆度
            if (circularity>=0.8) and (circularity<=0.95):
                return "circle"
            else:
                return "UNKNOW"
        else:
            return "UNKNOW"

    #---------作图---------------------------------------------------------
    """
    参数：frame为二值化后图像
    shapes
    """
    def draw_result(self,frame,shapes):
        cam_debug=frame.copy()
        for s in shapes:
            if s["type"]=="circle":
                cv2.circle(cam_debug,(s["cx"],s["cy"]),5,(0,0,255),-1)
            else:
                cv2.polylines(cam_debug,[s["approx"]],True)#绘制多边形
            #打印识别框
            cv2.putText(cam_debug,s["type"],(s["cx"]-40,s["cy"]-20),cv2.FONT_HERSHEY_SIMPLEX,0.7
                                (255,0,0),2)


        return cam_debug


#================================================
#入口
#================================================
def main():
    rclpy.init();
    node=ShapeRecognizer();
    try:
        rclpy.spin(node);
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()



if __name__=="__main__":
    main()