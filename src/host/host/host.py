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


class Host(Node):
    def __init__(self,name):
        super().__init__(name)
        #5个飞机的实例
        self.flights = [Flight(0),Flight(1),Flight(2),Flight(3),Flight(4)]
        #3个真车的实例
        self.cars = [Car(0),Car(1),Car(2)]

        #sub srv cli pub
        self.subs = []
        self.clis = []
        self.pubs = []
        for i in range(5):
            self.subs.append(
                self.create_subscription(FlightInfo, 'flightInfo_'+str(i), self.flights[i].getInfoCallback, 10)
            )
            self.pubs.append(
                self.create_publisher(FlightTarget, 'flightTarget_'+str(i), 10)
            )
            self.clis.append(
                self.create_client(FlightState,'flightState_'+str(i))
            )

        #全局时钟
        self.time = 0.0
        #起点坐标
        self.startPos = [0.0,0.0,0.0]
        #地面站状态机
        self.state = "init_powerUp"
        self.timer = self.create_timer(0.1,self.run)

    #由飞机信息确定真车
    def confirmTrueCar(self):
        pass

    #判断数字是否改变
    def tellNumChanged(self):
        pass

    #计算目标位置
    def getTargetPos(self):
        return []

    #判断是否能切换跟图模式
    def tellTargetFollowed(self,flight):
        pass

    #状态切换
    def newState(self,event):
        transitions = {
            ## state_from   event    state_to
            "init_powerUp":{"init_flightReady":"init_standby"},
            "init_standby":{"init_flyCmdInput":"init_waitNum"},
            "init_waitNum":{"init_newNum":"init_catchCar"},
            "init_catchCar":{"init_ready":"idle"},
            "idle":{"numChanged":"run"},
            "run":{"catched":"idle"},
        }

        if event in transitions[self.state]:
            self.state = transitions[self.state][event]
            print("new state:"+self.state)


    #对应状态执行
    def run(self):
        self.time = self.time + 0.1
        print('haoye')
        #计算真车的位置
        self.confirmTrueCar()
        #状态处理
        event = ""
        if self.state == "init_powerUp":
            #check number of flights
            sum = 0
            for i in range(5):
                if self.flights[i].state == "power_up":
                    sum = sum + 1
            #all flights power up
            if sum == 5:
                event = "init_flightReady"  

        elif self.state == "init_standby":
            #wait keyboard input 'takeoff'
            #send init position and change flight state
            req = [] #new flight event: takeoff 
            for i in range(5):
                self.clis[i].call_async(req[i])
            self.time = 0
            event = "init_flyCmdInput"

        elif self.state == "init_waitNum":
            #wait 50s
            if self.time > 50:
                event = "init_newNum"

        elif self.state == "init_catchCar":
            pass
        elif self.state == "idle":
            #判断数字是否改变，
            if self.tellNumChanged():
                #计算期望位置
                target = self.getTargetPos()
                #发送定位模式请求
                req = [] #new flight event: changeCar
                for i in range(5):
                    self.clis[i].call_async(req[i])
                #产生"numChanged"事件
                event = "numChanged"
            #没变啥也不干
        elif self.state == "run":
            #计算期望位置
            target = self.getTargetPos()
            #发送指令
            #统计跟上指令的飞机
            sum = 0
            for i in range(5):
                if self.flights[i].state == "follow_number":
                    sum = sum + 1
                else:
                    if self.tellTargetFollowed(self.flights[i]):
                        req = FlightState.Request()
                        req.event = "rightPos"
                        self.clis[i].call_async(req)
                    else:
                        self.pubs[i].publish(target[i])
            #判断是不是都跟上了
            if sum == 5:
                event = "catched"
        else:
            pass

        if not event == "":
            self.newState(event)
            

        


def main(args=None):
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = Host("HostNode")                          # 创建ROS2节点对象并进行初始化
    while rclpy.ok():
        #print('haoye1\n')
        if node.state == "init_standby":
            while input('ready to takeoff?[y]') != 'y':
                pass
        rclpy.spin_once(node)
        #print('haoye2\n')
        #node.run()
        #print('haoye3\n')
    #rclpy.spin(node)
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

