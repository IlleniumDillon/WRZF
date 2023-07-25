from enum import IntEnum

#----------------------------------------------------------------
# TPLink 通信协议相关定义
#----------------------------------------------------------------
class linkdef(IntEnum):
    MinPkSize = 8,    # 最小帧长度
    MaxPkSize = 128,  # 最大帧长度
    Header1   = 0xA5, # 帧起始符号1
    Header2   = 0x5A, # 帧起始符号2

# 遥测报文类型编码定义
class tlmtype(IntEnum):
    STD0    = 0x00,
    STD1    = 0x01,
    STD2    = 0x02,
    STD3    = 0x03,
    STD4    = 0x04,
    USR0    = 0x05,
    USR1    = 0x06,
    USR2    = 0x07,
    USR3    = 0x08,
    USR4    = 0x09,
    HIL     = 0x0A,
    TXT     = 0x0B,
    CODE    = 0x0C,
    BCST    = 0x20,
    GIMBAL  = 0x21,
    OBST    = 0x22

#----------------------------------------------------------------
# 飞行模式定义【MC】
#----------------------------------------------------------------
class flightmode(IntEnum):
    STAB    = 0 # 姿态模式
    ALTHOLD = 1 # 定高模式
    LOITER  = 2 # 定点模式
    AUTO    = 3 # 自主模式
    FOLLOW  = 4 # 跟踪模式
    STRIKE  = 5 # 打击模式
    OFFBOARD= 6 # 外部模式

#----------------------------------------------------------------
# 飞行模式子状态定义
#----------------------------------------------------------------
class flightstage(IntEnum):
    IDLE    = 0 # 空闲状态
    TAKEOFF = 1 # 自主起飞
    WPNAV   = 2 # 航点模式
    CIRCLE  = 3 # 盘旋模式
    GUIDED  = 4 # 引导模式
    RTL     = 5 # 返航模式
    LAND    = 6 # 降落模式
    TRANS   = 7 # 转换模式
    HOVER   = 8 # 悬停模式

#----------------------------------------------------------------
# 报文命令码字定义
#----------------------------------------------------------------
class linkcmd(IntEnum):
    MISSION_EDIT       = 0x10, # Modify Mission Info
    MISSION_QUERY      = 0x11, # Query Mission  data
    RCIN_UPDATE        = 0x20, # Update radio control data
    SERVO_OUT          = 0x21, # Set servo out data
    JOYSTICK_UPDATE    = 0x22, # Update joystick control data
    FOLLOW_TARGET      = 0x23, # Update Follow target information
    HEART_BEAT         = 0x30, # Heart beat for device  report
    PASS_THROUGH       = 0x31, # Send data to special port
    RTCM_UPDATE        = 0x32, # Send RTCM data to GNSS Receiver
    SET_FLIGHT_MODE    = 0x40, # Set flight mode
    DO_ACTION          = 0x41, # Do action by parameters
    ENTER_IAP          = 0xB1, # Set device to IAP mode
    ERASE_FLASH        = 0xB2, # Set device to erase flash
    DOWNLOAD_BIN       = 0xB3, # Set device to download firmware
    ENTER_UDISK        = 0xB4, # Set device to U-Disk mode
    SAVE_SETTING       = 0xCC, # Save setting data to flash
    READ_LOG_DATA      = 0xD0, # Read log data
    READ_LOG_META      = 0xD1, # Read log meta info
    CLEAR_LOG          = 0xD2, # Clear all log data
    READ_UOBJ_META     = 0xE0, # Read UAV Object meta data
    GET_FIELD_STR      = 0xE1, # Get field value by field name
    SET_FIELD_STR      = 0xE2, # Set field value by field name
    SET_SAVE_FIELD_STR = 0xE3, # Set&save field by field name
    GET_FIELD          = 0xF0, # Get field value by ID
    SET_FIELD          = 0xF1, # Set field value with response
    SET_SAVE_FIELD     = 0xF2, # Set and save field with response
    ZPACK_UPDATE       = 0xF3, # Send ZPACK data for HIL model
    OFFBOARD_REQ       = 0xF4  # Command request from offboard

#----------------------------------------------------------------
# DoAction操作子命令编码
#----------------------------------------------------------------
class actionsubcmd(IntEnum):
    TAKEOFF         = 0xA0,
    JUMP            = 0xA1,
    AUTH            = 0xA2,
    GUIDE           = 0xA3,
    TEST            = 0xA4,
    SET_FAKEGPS     = 0xA5,
    SET_HILMODE     = 0xA6,
    CALIB           = 0xA7,
    SET_ADDR        = 0xA8,
    FORMATION       = 0xA9,
    TOPLANDING      = 0xAA,
    POD_CTRL        = 0xAB,
    TRACE_OBJ       = 0xAC,
    TAG_MSG         = 0xAD,
    LAND            = 0xAE,
    HOVER           = 0xAF

#----------------------------------------------------------------
# 外部模式--请求类型
#----------------------------------------------------------------
class offboardReq(IntEnum):
    SET_POS  = 0x00 # 设置目标位置
    SET_VEL  = 0x01 # 设置目标速度
    SET_ATT  = 0x02 # 设置目标姿态
    GET_APV  = 0x03 # 查询位姿信息
    ENTER    = 0x04 # 切换到Offboard模式，接管无人机控制
    LEAVE    = 0x05 # 切换到Onboard 模式，退出外部控制
    LAND     = 0x06 # 执行末端降落
    GET_APV2 = 0x07 # 查询位姿信息2

#----------------------------------------------------------------
# 外部模式--指令坐标系
#----------------------------------------------------------------
class offboardCoord(IntEnum):
    FRU      = 0x00 # FRU坐标系-相对坐标
    NEU      = 0x01 # NEU坐标系-相对坐标
    WGS84    = 0x02 # WGS-84坐标系

#----------------------------------------------------------------
# PodCtrl操作子命令编码
#----------------------------------------------------------------
class podsubcmd(IntEnum):
    SNAP     = 0x00,
    ZOOM     = 0x01,
    ANGLE    = 0x02,
    RECORD   = 0x03,
    LIDAR    = 0x04,
    OSD      = 0x05,
    MOVE     = 0x06,
    VIDEOSRC = 0x07,
    TRACE    = 0x08,
    SETADDR  = 0x09,
    GETADDR  = 0x0A

class podMoveCode(IntEnum):
    STOP     = 0x00 # 停止
    UP       = 0x01 # 上移
    DOWN     = 0x02 # 下移
    LEFT     = 0x03 # 左移
    RIGHT    = 0x04 # 右移
    ZERO     = 0xFF # 归中

#----------------------------------------------------------------
#
#----------------------------------------------------------------