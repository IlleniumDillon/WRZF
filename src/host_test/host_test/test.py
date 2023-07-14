import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
import time

from host_interface.srv import FlightState
from host_interface.msg import FlightInfo
from host_interface.msg import FlightTarget
from host_interface.msg import ImageRst

from host.car import Car
from host.flight import Flight
from host.shiftPlan import ShiftPlan
from host.scheduler import Scheduler

class test(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pub = [] 
        for i in range(5):
            self.pub.append(self.create_publisher(FlightInfo, 'flightInfo_'+str(i), 10))
        self.timer = self.create_timer(0.1,self.run)

    def run(self):
        info = FlightInfo()
        info.pos = [20.0,35.0,5.0]

        rsts = []

        rst = ImageRst()
        rst.num = 1
        rst.x = 0.0
        rst.y = -1.0
        rst.size = 64*64.0
        
        rsts.append(rst)
        info.rsts = rsts
        self.pub[0].publish(info)
        rsts = []
        rst = ImageRst()
        info.pos = [10.0,25.0,5.0]
        rst.num = 1
        rst.x = 0.0
        rst.y = -1.0
        rst.size = 64*64.0
        rsts.append(rst)
        info.rsts = rsts
        self.pub[1].publish(info)
        rsts = []
        rst = ImageRst()
        info.pos = [15.0,20.0,5.0]
        rst.num = 1
        rst.x = -1.0
        rst.y = 0.0
        rst.size = 64*64.0
        rsts.append(rst)
        info.rsts = rsts
        self.pub[2].publish(info)
        rsts = []
        rst = ImageRst()
        info.pos = [30.0,15.0,5.0]
        rst.num = 3
        rst.x = 0.0
        rst.y = -1.0
        rst.size = 64*64.0
        rsts.append(rst)
        info.rsts = rsts
        self.pub[3].publish(info)
        rsts = []
        rst = ImageRst()
        info.pos = [35.0,10.0,5.0]
        rst.num = 3
        rst.x = -1.0
        rst.y = 0.0
        rst.size = 64*64.0
        rsts.append(rst)
        info.rsts = rsts
        self.pub[4].publish(info)


def main(args=None):#__node:=
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = test("testNode")                          # 创建ROS2节点对象并进行初始化
    while rclpy.ok():
        rclpy.spin_once(node)
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

