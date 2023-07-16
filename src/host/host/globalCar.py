'''
Author: IlleniumDillon 147900130@qq.com
Date: 2023-07-16 15:12:01
LastEditors: IlleniumDillon 147900130@qq.com
LastEditTime: 2023-07-16 16:29:26
FilePath: \WRZF\src\host\host\globalCar.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
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

class GlobalCar:
    def __init__(self):
        self.car = Car(-1)
        self.mergeTimes = 0
        self.flightSight = []
        self.time = -1.0
