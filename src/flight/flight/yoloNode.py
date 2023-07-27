'''
FilePath: flight.py
Author: Ballade-F     258300018@qq.com
Date: 2023-07-12 08:52:04
LastEditors: Please set LastEditors
LastEditTime: 2023-07-14 15:46:45
Copyright: 2023  All Rights Reserved.
Descripttion: 
'''
     
import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from std_msgs.msg import String                  # 字符串消息类型
import numpy as np


import torch

from host_interface.msg import ImageRst
from host_interface.msg import FlightInfo


import cv2
from ultralytics import YOLO

class YoloNode(Node):
    
    def __init__(self,name):
        super().__init__(name)          #父节点初始化

        # yolo检测相关参数，利用rqt reconfig动态调参
        self.declare_parameter('imgsize_min', "0.0")
        self.declare_parameter('imgsize_max', "0.2")
        self.declare_parameter('conf_min', "0.3")
        self.declare_parameter('conf_max', "1.0")
        self.imgSizeMin = float(self.get_parameter('imgsize_min').get_parameter_value().string_value)
        self.imgSizeMax = float(self.get_parameter('imgsize_max').get_parameter_value().string_value)
        self.confMin = float(self.get_parameter('conf_min').get_parameter_value().string_value)
        self.confMax = float(self.get_parameter('conf_max').get_parameter_value().string_value)
        self.get_logger().info('%f'%self.imgSizeMin)
        self.get_logger().info('%f'%self.imgSizeMax)
        self.get_logger().info('%f'%self.confMin)
        self.get_logger().info('%f'%self.confMax)
        self.flightID = 0
        # Load the YOLOv8 model，改成绝对路径
        self.model = YOLO('/home/zerone/best1.pt')
        # Open the video file
        self.cap = cv2.VideoCapture(0)
        self.capWidth = 640
        self.capHeight = 480
        self.get_logger().info("capture size set start!")
        self.cap.set(3,self.capWidth)
        self.cap.set(4,self.capHeight)
        self.get_logger().info("capture size set successfully!")

        self.imgInfo = []
        #注意这个是图像数据，只是数据类型是FlightInfo
        self.img_pub_ = self.create_publisher(FlightInfo, 'ImgInfo_%d' % self.flightID, 10)
        self.timer = self.create_timer(0.1, self.YoloUpdate)

        

    def YoloUpdate(self):
        # 一定是每帧都要发，即使什么都没检测到也得发，因为什么都没检测到也是地面站需要知道的一个信息
        self.getYoloResults()
        self.imgPub()


    #TODO:获取信息
    def getYoloResults(self):
        # 实时获取参数服务器的参数
        imgSizeMin = float(self.get_parameter('imgsize_min').get_parameter_value().string_value)
        imgSizeMax = float(self.get_parameter('imgsize_max').get_parameter_value().string_value)
        confMin = float(self.get_parameter('conf_min').get_parameter_value().string_value)
        confMax = float(self.get_parameter('conf_max').get_parameter_value().string_value)

        self.imgInfo = []

        # Read a frame from the video
        success, frame = self.cap.read()


        if success:
            # Run YOLOv8 inference on the frame
            results = self.model(frame)
            #annotated_frame = results[0].plot()

            #cv2.imshow("YOLOv8 Inference", annotated_frame)

            # Break the loop if 'q' is pressed
            # if cv2.waitKey(1) & 0xFF == ord("q"):
            #     pass
            #TODO:添加错误检测逻辑，排除掉误检测的
            if len(results[0].boxes.xywh) > 0:
                for i in range(len(results[0].boxes.xywh)):
                    rst = ImageRst()
                    rst.num = int(results[0].boxes.cls[i].item())
                    rst.conf = float(results[0].boxes.conf[i].item())
                    # 已归一化 右为x正，上为y正，图像中心为原点 x,y范围: -0.5 ~ 0.5
                    rst.x = -0.5 + (results[0].boxes.xyxyn[i][0].item() + results[0].boxes.xyxyn[i][2].item()) / 2
                    rst.y = 0.5 - (results[0].boxes.xyxyn[i][1].item() + results[0].boxes.xyxyn[i][3].item()) / 2
                    rst.size = (results[0].boxes.xywhn[i][2].item() + results[0].boxes.xywhn[i][3].item() ) / 2
                    rst.global_car_id = -1
                    # 只有置信概率和检测框大小都在范围内的，才会被添加
                    if 0.0 <= rst.conf<= 1.0 and 0.0 <= rst.size <= 0.5:
                        self.imgInfo.append(rst)


            #TODO:如果没检测到，发个甚末东西，是不是 self.imgInfo = [] 就行了


    # 给地面站发送自己的飞行信息
    def imgPub(self):
        # msg为飞机发给地面站的数据
        msg = FlightInfo()
        msg.rsts = self.imgInfo
        self.img_pub_.publish(msg)


            



def main(args=None):
    rclpy.init(args=args)  # ROS2 Python接口初始化
    node = YoloNode("YoloNode")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)
    node.destroy_node()  # 销毁节点对象
    rclpy.shutdown()  # 关闭ROS2 Python接口



