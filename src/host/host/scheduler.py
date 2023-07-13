import copy

from host.car import Car
from host.flight import Flight
from host.shiftPlan import ShiftPlan

class Scheduler:
    def __init__(self):
        self.flights = []
        for i in range(5):
            self.flights.append(Flight(i))
        self.cars = []
        for i in range(3):
            self.cars.append(Car(i))

        # 生成的调度计划表，存储对象
        self.shiftPlans = [ShiftPlan(-1, -1, -1, -1, -1)]

        # 还未分配的无人机，存储index
        self.unplanFlights = []

    # carNums 是 1*3的列表，存储三辆车的车顶数字
    def schedulerUpdate(self, carNums):
        # 初始化赋值
        self.unplanFlights = [0, 1, 2, 3, 4]
        for car in self.cars:
            car.prevFollowingFlights = copy.deepcopy(car.followingFlights)
            car.followingFlights = [-1, -1, -1]
            car.num = carNums[car.id]
            car.unsortNum = carNums[car.id]
            car.unsortFlights = []

        # 计算这一帧的编队情况
        planedFlights = []
        for car in self.cars:

            # 剔除已经规划过的
            for flight in planedFlights:
                self.unplanFlights.remove(flight)
            planedFlights = []

            # 判断没规划的无人机
            for flight in self.unplanFlights:

                if car.unsortNum > 0:
                    car.unsortFlights.append(flight)  # 分配飞机对象
                    planedFlights.append(flight)
                    car.unsortNum -= 1

                # 车的编号都已经分配完了就break
                else:
                    break

        # 清空调度计划表，准备往里填
        self.shiftPlans = []

        # 方位不需要从大往小排，哪个有空插哪个即可
        # 对于三辆车
        for car in self.cars:
            # 对于三个位置
            for direction in range(3):
                # 如果这个车的所有无人机都分配完了
                if len(car.unsortFlights) == 0:
                    break

                # 如果上一帧这里没有飞机，分配一个无人机到这个位置
                if car.prevFollowingFlights[direction] == -1:
                    # 先判断需要分配来的这个无人机是否上一帧和这一帧都在跟这个车，如果不是，就分配这个了
                    for flight in car.unsortFlights:
                        if flight not in car.prevFollowingFlights:
                            car.followingFlights[direction] = flight
                            car.unsortFlights.remove(flight)  # 删除指定索引的数据

                            # 更新调度表
                            self.shiftPlans.append(
                                ShiftPlan(flight,
                                          self.flights[flight].targetCar,
                                          self.flights[flight].targetDirection,
                                          car.id,
                                          direction))
                            # 更新飞机对象的属性
                            self.flights[flight].targetCar = car.id
                            self.flights[flight].targetDirection = direction

                            break
                        # 如果是，就什么都不做，再判断另外的是不，而这个无人机会在之后方位进入第二个分支
                        else:
                            pass


                # 如果上一帧这里有飞机
                else:
                    # 如果这一帧这个飞机还在跟这个车，就继续在这个位置，且不需要更新调度表
                    if car.prevFollowingFlights[direction] in car.unsortFlights:
                        car.followingFlights[direction] = car.prevFollowingFlights[direction]
                        car.unsortFlights.remove(car.prevFollowingFlights[direction]) # 删除指定值

                    # 如果这一帧这个飞机去跟其他车了，这个位置就空着了，就再分配个无人机过来
                    else:
                        # 先判断需要分配来的这个无人机是否上一帧和这一帧都在跟这个车，如果不是，就分配这个了
                        for flight in car.unsortFlights:
                            if flight not in car.prevFollowingFlights:
                                car.followingFlights[direction] = flight
                                car.unsortFlights.remove(flight) # 删除指定索引的数据

                                # 更新调度表
                                self.shiftPlans.append(
                                    ShiftPlan(flight,
                                              self.flights[flight].targetCar,
                                              self.flights[flight].targetDirection,
                                              car.id,
                                              direction))
                                # 更新飞机对象的属性
                                self.flights[flight].targetCar = car.id
                                self.flights[flight].targetDirection = direction

                                break
                            # 如果是，就什么都不做，再判断另外的是不，而这个无人机会在之后方位进入第二个分支
                            else:
                                pass



                # 如果到最后哪个都没进入，说明这里就不分配无人机了，保存原始的-1索引，表示这里没有无人机
