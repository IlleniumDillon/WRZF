from host.car import Car
from host.flight import Flight

import math

def carDistence(car1,car2):
    x = car1.pos[0] - car2.pos[0]
    y = car1.pos[1] - car2.pos[1]
    return math.sqrt(x*x+y*y)

def getGobalPos(flights,img2xyCoeff,tellSameCarThre):
    
    meargeCarList = [[],[],[],[]]
    lastNum = -1
    lastPos = []

    for car in allCarList:
        meargeCarList[car.num].append(car)

    rstCarList = []

    for carNumList in meargeCarList:
        # lastPos = []
        for car in carNumList:
        #     if len(lastPos) == 0:
            if  len(rstCarList) == 0 or rstCarList[-1].num != car.num:
                rstCarList.append(car)
            else:
                indx = -1
                minDis = 10000.0
                for i in range(len(rstCarList)):
                    car_last = rstCarList[i]
                    if carDistence(car_last,car) < minDis:
                        minDis = carDistence(car_last,car)
                        indx = i




    # for car in allCarList:
    #     if lastNum == -1:
    #         meargeCarList.append(car)
    #         lastNum = car.num
    #         lastPos = car.pos
    #     elif lastNum == car.num:
    #         disX = lastPos[0]-car.pos[0]
    #         disY = lastPos[1]-car.pos[1]
    #         dis = math.sqrt(disX*disX+disY*disY)
    #         if dis > tellSameCarThre:
    #             meargeCarList.append(car)
    #             lastNum = car.num
    #             lastPos = car.pos
    #         else:
    #             x = (meargeCarList[-1].pos[0] + car.pos[0]) / 2
    #             y = (meargeCarList[-1].pos[1] + car.pos[1]) / 2
    #             meargeCarList[-1].pos = [x,y]
    #     else:
    #         meargeCarList.append(car)
    #         lastNum = car.num
    #         lastPos = car.pos

    return rstCarList