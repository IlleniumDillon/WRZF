from enum import IntEnum

#----------------------------------------------------------------
# 通用设备类型枚举
#----------------------------------------------------------------
class udevtype(IntEnum):
    UNKNOWN = 0, # 未知类型
    COPTER  = 1, # 多旋翼机型
    PLANE   = 2, # 固定翼机型
    VTOL    = 3, # 垂起机型
    MRBOX   = 4, # 高精度盒子
    RADIO   = 5  # 智能电台