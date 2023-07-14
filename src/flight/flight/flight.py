'''
FilePath: flight.py
Author: Ballade-F     258300018@qq.com
Date: 2023-07-12 08:52:04
LastEditors: Please set LastEditors
LastEditTime: 2023-07-14 09:10:41
Copyright: 2023  All Rights Reserved.
Descripttion: 
'''
     
import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from std_msgs.msg import String                  # 字符串消息类型
import numpy as np

from host_interface.srv import FlightState
from host_interface.msg import FlightInfo
from host_interface.msg import FlightTarget
from host_interface.msg import ImageRst

class FlightNode(Node):
    
    def __init__(self,name):
        super().__init__(name)          #父节点初始化
        self.flightID = 0
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
        self.desPoint = [0.0, 0.0, 0.0]


        #控制
        self.imgPID = [PID(1,0,0,100,50,0,0.1),
                       PID(1,0,0,100,50,0,0.1),
                       PID(1,0,0,100,50,0,0.1)]

        self.pointPID = [PID(1,0,0,100,50,0,0.1),
                         PID(1,0,0,100,50,0,0.1),
                         PID(1,0,0,100,50,0,0.1)]


        self.flight_pub_ = self.create_publisher(FlightInfo, 'flightInfo_%d'%(self.flightID), 10)
        self.fsm_srv_ = self.create_service(FlightState, 'flightState_%d'%(self.flightID), self.fsmChangeSrv_callback)
        self.point_sub_ = self.create_subscription(FlightTarget, "flightTarget_%d"%(self.flightID), self.desPointSub_callback, 10)
        self.timer = self.create_timer(0.1,self.flightUpdate)


    #获取飞控信息
    def getFlightInfo(self):
        pass

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
        if self.fsm.getState() == 'follow_number':
            for i in range(3):
                self.imgPID[i].pidClear()
        elif self.fsm.getState() == 'follow_point':
            for i in range(3):
                self.pointPID[i].pidClear()
                self.desPoint[i] = request.pos[i]


        
        

    #接收地面站发送坐标
    def desPointSub_callback(self,msg):
        self.desPoint = msg.pos           
        self.desDir = msg.indx
        

    ##                           控制部分
    def imgCtrl(self):
        #根据队形位置、ID所分配的高度，选定期望像素坐标
        #TODO:


        for i in range(3):
            #self.imgPID[i].pidUpdate( , )#TODO:上面确定的des，图像给的坐标current
            pass
        return [self.imgPID[0].out,self.imgPID[1].out,self.imgPID[2].out]


    
    def pointCtrl(self):
        for i in range(3):
            self.pointPID[i].pidUpdate(self.desPoint[i],self.pos[i])
        return [self.pointPID[0].out,self.pointPID[1].out,self.pointPID[2].out]

    def send2Flight(self,outVel):
        pass

    def flightUpdate(self):
        self.flightPub()
        if self.fsm.getState() == 'follow_number':
            self.send2Flight(self.imgCtrl())
        elif self.fsm.getState() == 'follow_point' :
            self.send2Flight(self.pointCtrl())
        else:
            pass#异常




def main(args=None):
    rclpy.init(args=args)  # ROS2 Python接口初始化
    node = FlightNode("FlightNode")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)
    node.destroy_node()  # 销毁节点对象
    rclpy.shutdown()  # 关闭ROS2 Python接口



class PID:
    def __init__(self,pid_p,pid_i,pid_d,sum_max,out_max,out_min,T) :
        self.p=pid_p
        self.i=pid_i
        self.d=pid_d

        self.T = T

        self.error=0
        self.error_last=0
        self.error_sum=0
        self.sum_max=sum_max

        self.out=0
        self.out_max=out_max
        self.out_min=out_min

    def pidClear(self):
        self.error = 0
        self.error_last=0
        self.error_sum=0
        self.out=0

    def pidUpdate(self,des,current):
        self.error = des - current
        self.out = self.p*self.error + self.T*self.i*self.error_sum + (1/self.T)*self.d*(self.error-self.error_last)

        if self.out > self.out_max:
            self.out = self.out_max
        elif self.out < self.out_min:
            self.out = self.out_min
        else:
            pass#异常
        
        self.error_last = self.error
        self.error_sum += self.error

        if self.error_sum > self.sum_max:
            self.error_sum = self.sum_max




class FSM:
    def __init__(self):
        self.current_state = 'follow_number'
        self.last_state = 'follow_number'

    def transition(self,event):
        _transitions = {
            #state_from       event         state_to
            'follow_number' : {'changeCar' : 'follow_point'},
            'follow_point' : {'rightPos' : 'follow_number'}, 
        }

        if event in _transitions[self.current_state]:
            self.current_state = _transitions[self.current_state][event]

    def getState(self):
        return self.current_state