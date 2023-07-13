class Car:
    def __init__(self,id):
        #车上数字
        self.num = -1
        #车的坐标
        self.pos = [0.0,0.0]

        ###########################-added-#######################

        #         (0)             ^
        #                        /|\
        #        Car X            |  N
        #                         |
        #  (1)            (2)     |

        # 车里存储跟自己的无人机，依次为上述三个方位，方便查询，存储index
        self.followingFlights = [-1, -1, -1]
        self.prevFollowingFlights = [-1, -1, -1]

        self.unsortNum = -1
        self.unsortFlights = []  # 存储index

        # NOTE:id
        self.id = id