import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
import time

from host_interface.srv import FlightState
from host_interface.msg import FlightInfo
from host_interface.msg import FlightTarget
from host_interface.msg import ImageRst

from host.car import Car
from host.globalCar import GlobalCar

from host.flight import Flight
from host.shiftPlan import ShiftPlan
from host.scheduler import Scheduler

#from host.globalPos import *

img2xyCoeff = 1 
tellSameCarThre = 5#r
flightNum = 5
allCarListLen = 10
tellSameCarThre2 = 3
tellCarMoveThre = 0.5

import math

def carDistence(car1,car2):
    x = car1.pos[0] - car2.pos[0]
    y = car1.pos[1] - car2.pos[1]
    return math.sqrt(x*x+y*y)

def judgeCarMove(carList): #carList GlobalCar
    avgPos = [0.0,0.0]
    for i in range(allCarListLen):
        avgPos[0] += carList[i].car.pos[0]
        avgPos[1] += carList[i].car.pos[1]
    avgPos[0] /= allCarListLen
    avgPos[1] /= allCarListLen

    sumDis = 0.0

    for i in range(allCarListLen):
        dx = carList[i].car.pos[0] - avgPos[0]
        dy = carList[i].car.pos[1] - avgPos[1]
        dis = dx*dx+dy*dy
        sumDis += dis

    sumDis /= allCarListLen

    if sumDis > tellCarMoveThre:
        return True
    else:
        return False

class Host(Node):
    def __init__(self,name):
        super().__init__(name)
        #5个飞机的实例
        self.flights = [Flight(0),Flight(1),Flight(2),Flight(3),Flight(4)]
        #3个真车的实例
        self.cars = [Car(-1),Car(-1),Car(-1)]
        self.trueCarIndx = [-1,-1,-1]

        #sub srv cli pub
        self.subs = []
        self.clis = []
        self.pubs = []
        for i in range(flightNum):
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
        #last all car pos
        allCarList_last = [[],[],[],[],[],[]]        

    #由飞机信息确定真车
    def confirmTrueCar(self):
        #generate global car position
        #convert image info to xyz
        allCarList = []
        for i in range(flightNum):
            self.flights[i].carGlobal = []

            for j in range(len(self.flights[i].imgRsts)):
                coeffx = self.flights[i].imgRsts[j].x*(img2xyCoeff * self.flights[i].pos[2])
                coeffy = self.flights[i].imgRsts[j].y*(img2xyCoeff * self.flights[i].pos[2])
                car = GlobalCar()
                car.time = self.time
                car.car.num = self.flights[i].imgRsts[j].num
                car.car.pos = [self.flights[i].pos[0]+coeffx,self.flights[i].pos[1]+coeffy]
                flag = 0
                for k in range(len(allCarList)):
                    car_last = allCarList[k]
                    if car.car.num == car_last.car.num and \
                    carDistence(car.car,car_last.car) < tellSameCarThre and \
                    i not in car_last.flightSight:
                        self.flights[i].imgRsts[j].global_car_id = k
                        flag = 1
                        car_last.car.pos[0] += car.car.pos[0]
                        car_last.car.pos[1] += car.car.pos[1]
                        car_last.mergeTimes += 1
                        car_last.flightSight.append(i)
                        break
                        # if carDistence(car.car,car_last.car) < tellSameCarThre:
                        #     self.flights[i].ImageRst[j].global_car_id = k
                        #     flag = 1
                        #     break
                if flag == 0:
                    car.mergeTimes += 1
                    car.flightSight.append(i)
                    allCarList.append(car)
        
        if self.state == "init_catchCar" :
            ##这里和下一个分支是一样的
            allCarList_append_temp = [GlobalCar(),GlobalCar(),GlobalCar(),GlobalCar(),GlobalCar(),GlobalCar()]
            for i in range(len(allCarList)):
                car_now = allCarList[i]
                flag = 0
                for j in range(6):
                    if len(allCarList_last[j]) == 0:
                        break
                    car_last = allCarList_last[j][-1]
                    dis = carDistence(car_now,car_last)
                    if dis < tellSameCarThre2:
                        allCarList_append_temp[j] = car_now
                        flag = 1
                        break
                if flag == 0:
                    for j in range(6):
                        if len(allCarList_last[j]) == 0:
                            allCarList_append_temp[j] = car_now
            for i in range(6):
                if allCarList_append_temp[i].time > 0:
                    allCarList_last[i].append(allCarList_append_temp[i])
                    if len(allCarList_last[i]) > allCarListLen :
                        allCarList_last[i].pop(0)
            ##上面和下一个分支是一样的
                        if judgeCarMove(allCarList_last[i]):
                            #这里认为第一个动的就是第一辆真车编号是0
                            for j in range(3):
                                if self.trueCarIndx[j] == -1:
                                    self.trueCarIndx[j] = i
            for i in range(3):
                if self.trueCarIndx[i] == -1:
                    self.cars[i].num = allCarList_last[self.trueCarIndx[i]][-1].num
                    self.cars[i].pos = allCarList_last[self.trueCarIndx[i]][-1].pos
                    self.cars[i].id = i

        else:
            allCarList_append_temp = [GlobalCar(),GlobalCar(),GlobalCar(),GlobalCar(),GlobalCar(),GlobalCar()]
            for i in range(len(allCarList)):
                car_now = allCarList[i]
                flag = 0
                for j in range(6):
                    if len(allCarList_last[j]) == 0:
                        break
                    car_last = allCarList_last[j][-1]
                    dis = carDistence(car_now,car_last)
                    if dis < tellSameCarThre2:
                        allCarList_append_temp[j] = car_now
                        flag = 1
                        break
                if flag == 0:
                    for j in range(6):
                        if len(allCarList_last[j]) == 0:
                            allCarList_append_temp[j] = car_now

            missFlag = -1
            for i in range(6):
                if allCarList_append_temp[i].time > 0:
                    allCarList_last[i].append(allCarList_append_temp[i])
                    if len(allCarList_last[i]) > allCarListLen :
                        allCarList_last[i].pop(0)
                    if i in self.trueCarIndx:
                        self.cars[i].num = allCarList_last[self.trueCarIndx[i]][-1].num
                        self.cars[i].pos = allCarList_last[self.trueCarIndx[i]][-1].pos
                        self.cars[i].id = i
                else:
                    if i in self.trueCarIndx:
                        missFlag = trueCarIndx.indx(i)

            if missFlag == 0:
                self.cars[0].num = 5 - self.cars[1].num - self.cars[2].num
                dx = self.cars[1].pos[0] - self.cars[2].pos[0]
                dy = self.cars[1].pos[1] - self.cars[2].pos[1]
                self.cars[0].pos[0] = self.cars[1].pos[0] + dx
                self.cars[0].pos[1] = self.cars[1].pos[1] + dy
            elif missFlag == 1:
                self.cars[1].num = 5 - self.cars[0].num - self.cars[2].num
                self.cars[1].pos[0] = (self.cars[0].pos[0]+self.cars[2].pos[0]) / 2
                self.cars[1].pos[1] = (self.cars[0].pos[1]+self.cars[2].pos[1]) / 2
            elif missFlag == 2:
                self.cars[2].num = 5 - self.cars[0].num - self.cars[1].num
                dx = self.cars[0].pos[0] - self.cars[1].pos[0]
                dy = self.cars[0].pos[1] - self.cars[1].pos[1]
                self.cars[2].pos[0] = self.cars[1].pos[0] - dx
                self.cars[2].pos[1] = self.cars[1].pos[1] - dy

        for i in range(len(allCarList)):
            allCarList[i].car.pos[0] /= allCarList[i].mergeTimes
            allCarList[i].car.pos[1] /= allCarList[i].mergeTimes
            print(allCarList[i].car.num)
            print(allCarList[i].car.pos)
        #allCarPos = getGobalPos(self.flights,img2xyCoeff,tellSameCarThre)
        # for car in allCarPos:
        #     print(car.num)
        #     print(car.pos)
        while 1:
            pass
        #TODO: tell true car and confirm car number
        #allCarPos_last = allCarPos

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
        #计算真车的位置
        if self.state == "init_catchCar" or self.state == "idle" or self.state == "run" :
            self.confirmTrueCar()
        #状态处理
        event = ""
        if self.state == "init_powerUp":
            #check number of flights
            sum = 0
            for i in range(flightNum):
                if self.flights[i].state == "power_up":
                    sum = sum + 1
            #all flights power up
            if sum == flightNum:
                event = "init_flightReady"  

        elif self.state == "init_standby":
            #wait keyboard input 'takeoff'
            #send init position and change flight state
            req = [] #new flight event: takeoff 
            for i in range(flightNum):
                self.clis[i].call_async(req[i])
            self.time = 0
            event = "init_flyCmdInput"

        elif self.state == "init_waitNum":
            #wait 50s
            if self.time > 50:
                event = "init_newNum"

        elif self.state == "init_catchCar":
            #TODO: 
            if -1 not in self.trueCarIndx:
                #计算期望位置
                target = self.getTargetPos()
                #发送定位模式请求
                req = [] #new flight event: catchCar
                for i in range(flightNum):
                    self.clis[i].call_async(req[i])
                event = "init_ready"

        elif self.state == "idle":
            #判断数字是否改变，
            if self.tellNumChanged():
                #计算期望位置
                target = self.getTargetPos()
                #发送定位模式请求
                req = [] #new flight event: changeCar
                for i in range(flightNum):
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
            for i in range(flightNum):
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
            if sum == flightNum:
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

