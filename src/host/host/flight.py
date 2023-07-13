from host_interface.srv import FlightState
from host_interface.msg import FlightInfo
from host_interface.msg import FlightTarget
from host_interface.msg import ImageRst

class Flight:
    def __init__(self,id):
        #飞行姿态、速度、位置
        self.rpy = [0.0,0.0,0.0]
        self.pos = [0.0,0.0,0.0]
        self.vel = [0.0,0.0,0.0]
        #图像信息
        self.imgRsts = []
        #飞机当前状态
        self.state = ""

        #########################-added-#########################
        # NOTE:添加上了在跟哪辆车和在哪个位置
        self.targetCar = -1
        self.targetDirection = -1
        # NOTE:添加上了跟坐标模式下的目标坐标
        self.targetPos = [0.0, 0.0, 0.0]
        # TODO:添加上状态机类
        self.flyState = None

        # NOTE:id
        self.id = id

    #订阅飞机各种信息的回调
    def getInfoCallback(self,data):
        self.rpy = data.rpy
        self.pos = data.pos
        self.vel = data.vel
        self.imgRsts = data.rsts
        self.state = data.state
        print('get flight info')
        print(self.state)
        pass
    #改飞机状态
    def newState():
        pass
    #改飞机目标点
    def newTarget():
        pass

