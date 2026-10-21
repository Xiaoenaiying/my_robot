import sys
# 强行把系统自带的正确路径插入到搜索列表的最前面
sys.path.insert(0, '/usr/lib/python3/dist-packages')
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from typing import Optional, Dict, Tuple
import os
import cv2
import hbm_runtime
print("实际加载的 hbm_runtime 路径是:", hbm_runtime.__file__)
import utils.preprocess_utils as preprocess
import utils.postprocess_utils as postprocess
import utils.common_utils as common
import utils.draw_utils as draw
import numpy as np

class yolo_detector_node(Node):
    def __init__(self):
        super().__init__("yolo_detector_node")
        self.pkg_name=get_package_share_directory("my_robot_yolo_detection")
        model_path=os.path.join(self.pkg_name,"models","yolo_v8purple_block_bayese_640x640_nv12.bin")
        self.model=hbm_runtime.HB_HBMRuntime(model_path)
        #----------------配置字典顶层---------------------------------------------------
        self.model_name=self.model.model_names[0]
        #----------------配置字典内层---------------------------------------------------
        self.model_inputname=self.model.input_names[self.model_name]
        #---------------配置字典输出名称------------------------------------------------
        self.model_outputname=self.model.output_names[self.model_name]
        #---------------配置字典输入形状------------------------------------------------
        self.model_inputshape=self.model.input_shapes[self.model_name]
        #--------------配置反量化(换回真实的数值)----------------------------------------
        self.model_outputquants=self.model.output_quants[self.model_name]
        #--------------订阅者节点创建---------------------------------------------------
        #订阅RGB相机
        self.create_subscription(Image,"Image_raw",self.yolo_callback,3)
        #订阅深度相机
        self.create_subscription(Depth,"depth_raw",self.depth_callback,3)
        #-------------输入形状的长宽----------------------------------------------------
        self.input_H=self.model_inputshape[self.model_inputname[0]][2]
        self.input_W=self.model_inputshape[self.model_inputname[0]][3]

        #------------图像缩放策略-------------------------------------------------------
        self.resize_type=1
        
        #------------三步检测的步长-----------------------------------------------------
        self.strige=[8,16,32]
        #------------每步检测下特征图的网格数--------------------------------------------
        self.anchor_size=[80,40,20]

        #------------sigmoid 的反函数---------------------------------------------------
        self.score_thres = 0.25          #用户层面的置信度阈值
        self.conf_thres_raw=-np.log(1/self.score_thres-1)  #把概率阈值转回 logits 阈值

        #------------nms阈值去重复框----------------------------------------------------
        self.nms_thresh=0.45

        #DFL解码固定权重
        self.reg=16#回归张量数量
        self.weights_static = np.arange(self.reg, dtype=np.float32)[np.newaxis, np.newaxis, :]#DFL解码常用权重

        #-------------模型能识别的尺寸---------------------------------------------------
        self.input_W = 640
        self.input_H = 640

        #yolo识别后图像发布,发布者工作
        self.yolo_debug=self.create_publisher(Image,"yolo_debug",3)

        #深度图像保存
        self.depth=None

        #实例化cvbrige用于图像转换
        self.brige=CvBridge()

        print("--- 模型元数据开始 ---")
        print("model_name:", self.model_name)
        print("input_names:", self.model_inputname)
        print("input_shapes:", self.model_inputshape)
        print("output_names:", self.model_outputname)
        for n in self.model_outputname:
            print(n, "shape:", self.model.output_shapes[self.model_name][n])
        print("--- 模型元数据结束 ---")

    """
    @brief:RGB相机订阅者回调
    """
    def yolo_callback(self,msg:Image):
        try:
            frame=self.brige.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().info(f"图像转换失败的原因是{e}")
            return
        self.yolo_set_scheduling_params(priority=1,bpu_cores=[2])
        pre_camera=self.yolo_pre_process(frame)
        yolo_run_camera=self.yolo_foward(pre_camera)
        boxes,cls_ids,scores=self.yolo_post_process(yolo_run_camera,self.input_W,self.input_H)
        coco_path=os.path.join(self.pkg_name,"my_yolo_checkself","coco_classes.names")
        coco_names=common.load_class_names(coco_path)
        yolo_img=draw.draw_boxes(frame,boxes,cls_ids,scores,coco_names,common.rdk_colors)
        self.yolo_debug.publish(self.brige.cv2_to_imgmsg(yolo_img,"bgr8"))

        if self.depth is None:
            print(f"图像数据没有收到")

        #3D坐标计算
        #置信度最高
        max_idx=np.argmax(scores)
        box=boxes[max_ids]

        #计算原始的图像坐标中心点(u,v)
        u=int((box[0]+box[2])/2.0)
        v=int((box[1]+box[3])/2.0)

        #中心点所对应的深度值
        raw_depth=self.depth[v,u]
        
        print(f"调试: 中心点({u},{v}), 深度原始值 = {raw_depth}")

        if raw_depth>0:
            z=float(raw_depth)/1000.0

            if z>0.1 and z<1.5:
                # 640x400 下的精准内参
                fx = 345.20
                fy = 383.51
                cx = 320.54
                cy = 201.14

                X=(u-cx)*z/fx
                Y=(v-cy)*z/fy
                self.get_logger().info(f"purple_block{X},{Y},{z},{raw_depth}")

    """
    @brief:深度相机订阅者回调
    """
    def depth_callback(self,msg:Image):
        try:
            self.depth=self.brige.imgmsg_to_cv2(msg,desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().info(f"图像转换失败的原因是{e}")
            return 


    """
    @brief:用于设置优先级和BPU推理核心
    @param:propety[0~255]优先级设置，BPU_coreBPU推理的运行在设备具体核心
    """    
    def yolo_set_scheduling_params(self,priority:Optional[int]=None,bpu_cores:Optional[list]=None):
        kwargs={}
        if priority is not None:
            kwargs["priority"]={self.model_name:priority}
        if bpu_cores is not None:
            kwargs["bpu_cores"]={self.model_name:bpu_cores}
        if kwargs:
            self.model.set_scheduling_params(**kwargs)

    """
    @breif：缩放到模型输入尺寸，转成NV12格式，内存拼接(预处理)
    @param:img输入的原始画面张量信息
    @return:该模型下的输入名称对应的画面
    """
    def yolo_pre_process(self,img:np.ndarray):
        #------------缩放到模型输入尺寸（图像）-------------------------------------------
        resize_img=preprocess.resized_image(img,self.input_H,self.input_W,self.resize_type)
        #------------BGR转NV12平面(y存放亮度，uv存色度)-----------------------------------
        y,uv=preprocess.bgr_to_nv12_planes(resize_img)
        #------------保证数据类型为uint8-------------------------------------------------
        y=y.astype(np.uint8)
        uv=uv.astype(np.uint8)
        #-----------NV12内存拼接---------------------------------------------------------
        nv12=np.concatenate((y.reshape(-1),uv.reshape(-1)),axis=0) #一维数组
        nv12=nv12.reshape((1,self.input_H*3//2,self.input_W,1))

        return{
            self.model_name:{
                self.model_inputname[0]:nv12
            }
        }

    """
    @brief:处理输入的张量信息
    @param:input_tensor（字典嵌套）输入的张量信息
    @return:
    """
    def yolo_foward(self,input_tensor:Dict[str,Dict[str,np.ndarray]])->Dict[str,np.ndarray]:
        outputs=self.model.run(input_tensor)
        return outputs[self.model_name]

    """
    @brief:后处理,反量化用于后面处理，分类与回归分支分别处理结果，NMS去重，图框坐标
    @param:outputs是经过模型运算后的图像张量
    @param:img_w图像的宽
    @param:img_h图像的高
    @return:
    """
    def yolo_post_process(self,outputs:Dict[str,np.ndarray],img_w:int,img_h:int):
        #列表存储识别框，物品ID，分数
        all_dbboxes=[]
        all_ids=[]
        all_scores=[]   

        #----------反量化，统一标尺,距离回到现实--------------
        fp32_outputs=postprocess.dequantize_outputs(outputs,self.model_outputquants)
        #----------分类分支筛选格式
        for i,(strige,anchor_size) in enumerate(zip(self.strige,self.anchor_size)):
            #---------------每两个一组
            cls_key=self.model_outputname[2*i]    #分类
            box_key=self.model_outputname[2*i+1]  #回归

            #---------------按置信度阈值找候选目标,得到得分，物品ID，目标索引           
            scores, ids, valid_indices=postprocess.filter_classification(fp32_outputs[cls_key],self.conf_thres_raw)

            #---------------解码边界框
            dbboxes=postprocess.decode_boxes(fp32_outputs[box_key],valid_indices,anchor_size,
                                             strige,self.weights_static)

            #选择预测分支结果,得到图框，物品ID，得分
            all_dbboxes.append(dbboxes)
            all_ids.append(ids)
            all_scores.append(scores)
        #---------暂时存储识别框，物体ID，分数
        dbboxes=np.concatenate(all_dbboxes,axis=0)
        ids=np.concatenate(all_ids,axis=0)
        score=np.concatenate(all_scores,axis=0)

        #-----------NMS应用（去重复识别）,得到是没有重复的索引
        NMS=postprocess.NMS(dbboxes,score,ids,1)

        #-----------坐标映射回原图
        xyxy=postprocess.scale_coords_back(dbboxes,img_w,img_h,self.input_W,self.input_H,self.resize_type)

        #NMS筛选抑制重复
        cls_id=ids[NMS]
        scores=score[NMS]

        return xyxy,cls_id,scores


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
