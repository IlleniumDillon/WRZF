'''
FilePath: flight.py
Author: Ballade-F     258300018@qq.com
Date: 2023-07-12 08:52:04
LastEditors: Please set LastEditors
LastEditTime: 2023-07-26 10:03:01
Copyright: 2023  All Rights Reserved.
Descripttion: 
'''
     
import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from std_msgs.msg import String                  # 字符串消息类型
import numpy as np

from flight.fsm import FSM
from flight.pid import PID

from host_interface.srv import FlightState
from host_interface.msg import FlightInfo
from host_interface.msg import FlightTarget
from host_interface.msg import ImageRst

from flight.UAVItem import *
from flight.LinkDef import *

class FlightNode(Node):
    
    def __init__(self,name):
        super().__init__(name)          #父节点初始化

        #可调常量参数
        self.declare_parameter('flight_id',"0")
        self.declare_parameter('com_number',"/dev/ttyTHS0")
        self.flightID = int(self.get_parameter('flight_id').get_parameter_value().string_value)
        self.get_logger().info('%d'%self.flightID)
        self.number_height = 5
        self.height_differ = 2
        self.height_error = 0.5
        self.pos_error = 1
        self.pix_error = 10

        self.declare_parameter('imgx_kp',"0.0")
        self.declare_parameter('imgy_kp',"0.0")
        self.declare_parameter('posx_kp',"0.0")
        self.declare_parameter('posy_kp',"0.0")
        self.declare_parameter('posz_kp',"0.0")

        #起飞阶段相关参数
        self.first_height = 10
        self.first_height_differ = 3

        #

        

        #飞行信息
        self.rpy = [0.0,0.0,0.0]
        self.pos = [0.0,0.0,0.0]
        self.vel = [0.0,0.0,0.0]
        #图像信息
        self.imgInfo = []
        #状态机
        self.fsm = FSM()
        

        self.desCar = 0
        self.desDir = 0
        self.desPixX = 0
        self.desPixY = 0 
        self.desPoint = [0.0, 0.0, 0.0]



        #控制
        self.imgPID = [PID(1,0,0,100,5,-5,0.1),
                       PID(1,0,0,100,5,-5,0.1)]

        self.pointPID = [PID(1,0,0,100,50,-50,0.1),
                         PID(1,0,0,100,50,-50,0.1),
                         PID(1,0,0,100,50,-50,0.1)]


        self.flight_pub_ = self.create_publisher(FlightInfo, 'flightInfo_%d'%(self.flightID), 10)
        self.fsm_srv_ = self.create_service(FlightState, 'flightState_%d'%(self.flightID), self.fsmChangeSrv_callback)
        self.point_sub_ = self.create_subscription(FlightTarget, "flightTarget_%d"%(self.flightID), self.desPointSub_callback, 10)
        self.imh_sub_ = self.create_subscription(FlightInfo,'ImgInfo_%d' % self.flightID, self.imgProcess_callback,10)
        self.timer = self.create_timer(0.1,self.flightUpdate)
        #与飞控相关
        self.comNumber = self.get_parameter('com_number').get_parameter_value().string_value
        self.uav = uavitem(self.comNumber)
        self.uav.Open()
        self.get_logger().info('%s'%self.comNumber)
        self.get_logger().info('%d'%self.uav.IsConnected())
        self.uav.Link.SendCmdOffboardEnter()



    #TODO:获取飞控信息
    def getFlightInfo(self):
        #self.get_logger().info('%f'%self.uav.Lat)
        self.pos[0] = self.uav.Lng
        self.pos[1] = self.uav.Lat
        self.pos[2] = self.uav.Alt

        pass

    #TODO:调用图像处理函数
    def imgProcess_callback(self,msg):
        self.imgInfo = msg.rsts

    #给地面站发送自己的飞行信息
    def flightPub(self):
        #msg为飞机发给地面站的数据
        msg = FlightInfo()
        msg.rpy = self.rpy
        msg.pos = self.pos
        msg.vel = self.vel
        msg.rsts = self.imgInfo
        msg.state = self.fsm.getState()
        self.flight_pub_.publish(msg)

    #接受地面站让飞机改变状态
    def fsmChangeSrv_callback(self,request,response):
        self.fsm.transition(request.event)
        #从跟数字切为上升高度跟数字
        if self.fsm.getLastState() == 'follow_number':
            for i in range(2):
                self.pointPID[i].pidClear()  
        if self.fsm.getLastState() == 'power_up':
            self.uav.Link.SendCmdOffboardEnter()
            self.desPoint = request.pos
        response.flag = 1
        return response
        

    #接收地面站发送坐标
    def desPointSub_callback(self,msg):
        self.desPoint = msg.pos


        self.desDir = msg.indx
        

    ##                           控制部分
    def imgCtrl(self):
        #根据队形位置，选定期望像素坐标
        #TODO:等标定
        if self.desDir == 0 :
            #self.self.desPixX = 
            #self.self.desPixY = 
            pass
        elif self.desDir == 1 :
            #self.self.desPixX = 
            #self.self.desPixY = 
            pass
        elif self.desDir == 2 :
            #self.self.desPixX = 
            #self.self.desPixY = 
            pass
        else :
            #异常
            pass
        self.imgPID[0].p = float(self.get_parameter('imgx_kp').get_parameter_value().string_value)
        self.imgPID[1].p = float(self.get_parameter('imgy_kp').get_parameter_value().string_value)
        self.pointPID[2].p = float(self.get_parameter('posz_kp').get_parameter_value().string_value)


        self.desPixX = 0
        self.desPixY = 0

        #follow the nearst car
        min_distance = 1
        car_id = -1
        for car in range(len(self.imgInfo)):
            dis = self.imgInfo[car].x * self.imgInfo[car].x + self.imgInfo[car].y * self.imgInfo[car].y
            if dis <min_distance:
                min_distance = dis
                car_id = car
        if car_id == -1:
            return [0,0,0]

        #水平控制用图像，高度控制用点位
        self.desPoint[2] = self.number_height

        self.pointPID[2].pidUpdate(self.desPoint[2],self.pos[2])
        self.imgPID[0].pidUpdate(self.desPixX ,self.imgInfo[car_id].x)#TODO:如果看到不止一个数字怎么办
        self.imgPID[1].pidUpdate(self.desPixY ,self.imgInfo[car_id].y)
       
        return [self.imgPID[1].out,self.imgPID[0].out,self.pointPID[2].out]


    
    def pointCtrl(self):
        #根据飞机id差高  5 7 9 11 13
        self.desPoint[2] = self.number_height + self.height_differ*self.flightID
        for i in range(3):
            self.pointPID[i].pidUpdate(self.desPoint[i],self.pos[i])
        return [self.desPoint[0],self.desPoint[1],self.desPoint[2]]

    def send2Flight(self,outVel_Pos,posFlag):
        #TODO:注意坐标系转化,注意速度模式和位置模式
        if posFlag:
            self.uav.Link.SendCmdOffboardSetPos(offboardCoord.WGS84, 1000, self.desPoint[1], self.desPoint[0], self.desPoint[2], 0)
            self.get_logger().info('[%f]'%self.desPoint[0])
            self.get_logger().info('[%f]'%self.desPoint[1])
            self.get_logger().info('[%f]'%self.desPoint[2])
        else :
            self.uav.Link.SendCmdOffboardSetVel(offboardCoord.FRU, 1000, outVel_Pos[0], outVel_Pos[1],outVel_Pos[2], 0)
            self.get_logger().info('[%f]'%outVel_Pos[0])
            self.get_logger().info('[%f]'%outVel_Pos[1])


    def flightUpdate(self):
        self.getFlightInfo()
        self.flightPub()
        if self.fsm.getState() == 'follow_number':
            # self.desPoint[2] = self.number_height 
            self.send2Flight(self.imgCtrl(),False)

        elif self.fsm.getState() == 'follow_number_high' :
            self.desPoint[2] = self.number_height + self.height_differ*self.flightID
            self.send2Flight(self.imgCtrl(),False)
            #从高处跟数字 切为 跟坐标
            if self.fsm.getLastState() == 'follow_number' and abs(self.pointPID[2].error) < self.height_error:
                self.fsm.transition('rightHeight')
                for i in range(2):
                    self.imgPID[i].pidClear()
            elif (self.fsm.getLastState() == 'follow_point' or self.fsm.getLastState() == 'go_start')\
                and abs(self.imgPID[0].error)<self.pix_error and abs(self.imgPID[1].error)<self.pix_error:
                self.fsm.transition('rightPix')
            else:
                pass

        elif self.fsm.getState() == 'follow_point' :
            self.send2Flight(self.pointCtrl(),True)

        elif self.fsm.getState() == 'power_up':
            #TODO:上电，等待
            pass
        elif self.fsm.getState() == 'go_up_height': 
            #TODO：起飞到特定高度 还需确认接口使用无误
            self.send2Flight(self.pointCtrl(),True)
            # self.uav.Link.SendCmdTakeoff()
            if abs(self.pointPID[2].error) < self.height_error:
                self.fsm.transition('rightHeight')
           
        elif self.fsm.getState() == 'go_start': 
            #TODO:到达起飞点，等待
            # self.desPoint[2] = self.first_height + self.flightID * self.first_height_differ
            self.send2Flight([self.desPoint[0],self.desPoint[1],self.desPoint[2]],True)
            # if abs(self.pos[0]-self.desPoint[0]) < self.pos_error and \
            # abs(self.pos[0]-self.desPoint[0]) < self.pos_error and \
            # abs(self.pos[0]-self.desPoint[0]) < self.pos_error :
            #     self.fsm.transition('catchCar')

        else:
            pass#异常




def main(args=None):
    rclpy.init(args=args)  # ROS2 Python接口初始化
    node = FlightNode("FlightNode")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)
    node.destroy_node()  # 销毁节点对象
    rclpy.shutdown()  # 关闭ROS2 Python接口




