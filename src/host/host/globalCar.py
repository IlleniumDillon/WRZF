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
