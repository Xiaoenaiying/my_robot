import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO          # YOLO 模型加载和推理
from ament_index_python.packages import get_package_share_directory
import os
import cv2

class yolo_detector_node(Node):
    def __init__(self):
        super().__init__("yolo_detector_node")
        pkg_name=get_package_share_directory("my_robot_yolo_detection")
        model_path=os.path.join(pkg_name,"models","yolov8n.pt")
        self.model=YOLO(model_path)
        self.create_subscription(Image,"Image_raw",self.yolo_callback,3)
        #yolo识别后图像发布
        self.yolo_debug=self.create_publisher(Image,"yolo_debug",3)
        #实例化cvbrige用于图像转换
        self.brige=CvBridge()

    def yolo_callback(self,msg):
        self.get_logger().info("模型开始处理画面")
        try:
            img=self.brige.imgmsg_to_cv2(msg,desired_encoding="bgr8")
            image=img.copy()
        except Exception as e:
            self.get_logger().info(f"当前图像格式转换错误原因是{e}")
            return 
        results=self.model(image,verbose=False)#得到Results结果

        for r in results:#遍历每张图
            for box in r.boxes:#遍历每张图的检测框
                #调用boxes里的方法
                x1,y1,x2,y2=map(int,box.xyxy[0].tolist())
                Confidence_level=box.conf[0]
                if Confidence_level<0.5:
                    continue
                cls_id=int(box.cls[0])
                type=self.model.names[cls_id]
                cx=(x1+x2)/2
                cy=(y1+y2)/2
                self.get_logger().info(f"中心点为({cx},{cy}),置信度为{Confidence_level},识别类型为{type}")
                cv2.rectangle(image,(x1,y1),(x2,y2),(0,0,255),2)
                cv2.putText(image,f"{type} {Confidence_level}",(x1,y1),cv2.FONT_HERSHEY_SIMPLEX,1.0,(0,255,0),2)
            self.yolo_debug.publish(self.brige.cv2_to_imgmsg(image,"bgr8"))
        

def main():
    rclpy.init()
    try:
        node=yolo_detector_node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__=="__main__":
    main()
