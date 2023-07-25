
from flight.Telemetry import *
from flight.LinkUtils import *
from flight.TPLink import *
# 描述飞机对象类

class uavitem(object):
    def __init__(self, port_num):
        self.Name    = '' # 对象名称
        self.Link    = tplink(port_num, self)

        #region 姿态航向
        self.Roll    = 0.0 # 滚转角 deg
        self.Pitch   = 0.0 # 俯仰角 deg
        self.Yaw     = 0.0 # 偏航角 deg (机头指向)
        self.Course  = 0.0 # 航迹角 deg (运动方向)
        #endregion

        self.Lat     = 0.0 # 当前纬度 deg
        self.Lng     = 0.0 # 当前经度 deg

        #region 高度信息
        self.Alt     = 0.0 # 相对高度 meter(相对Home点)
        self.AGL     = 0.0 # 对地高度 meter
        self.GPSAlt  = 0.0 # GPS高度  meter
        self.BaroAlt = 0.0 # 气压高度 meter
        #endregion

        #region 吊舱载荷
        self.GimbalRoll  = 0     # Gimbal ENC roll angle in deg
        self.GimbalPitch = 0     # Gimbal ENC pitch angle in deg
        self.GimbalYaw   = 0     # Gimbal ENC yaw angle in deg
        self.GimbalLaserDist = 0 # 激光测距 距离(米)
        self.GimbalObjLng = 0    # 目标指示 纬度
        self.GimbalObjLat = 0    # 目标指示 精度
        self.GimbalZoomLevel = 0 # 变倍倍数
        #endregion

        self.IsReady     = False
        self.IsOffGnd    = False
        self.IsLocked    = False
        self.IsVibeWarn  = False
        self.IsLogOk     = False
        self.IsBatFail   = False

        self.Speed       = 0
        self.NavSpd      = 0
        self.RCState     = 0
        self.FlightMode  = 0
        self.FlightStage = 0
        self.BaroAlt     = 0
        self.AGL         = 0
        self.Bat1Volt    = 0
        self.Bat2Volt    = 0


    #--------------------------------------------------------------------------
    # 公共成员函数
    #--------------------------------------------------------------------------
    def Open(self):
        if self.Link is None:
            return False

        if(self.Link.IsOpened()):
            self.Link.SendCmdHeartBeat()
            return True

        return False

    # 联机状态指示
    def IsConnected(self):
        if self.Link is None:
            return False

        return self.Link.IsOpened()

    # 标准遥测信息更新处理
    def OnStdTlmRcvEvent(self, tlm):
        self.Roll  = 0.01 * tlm.Roll
        self.Pitch = 0.01 * tlm.Pitch
        self.Lat   = 1e-7 * tlm.Lat
        self.Lng   = 1e-7 * tlm.Lng

        if isinstance(tlm, StdTlm0) :
            self.Speed       = tlm.Speed * 0.01
            self.NavSpd      = tlm.TargetSpd * 0.01
            self.RCState     = tlm.RCState
            self.FlightMode  = tlm.FlightMode >> 4
            self.FlightStage = tlm.FlightMode & 0x0F
            self.BaroAlt     = tlm.BaroAlt * 0.01
            self.AGL         = tlm.AGL * 0.01 # 转换为m
            self.Bat1Volt    = 0.01 * tlm.Volt1 # 电池电压[V]
            self.Bat2Volt    = 0.01 * tlm.Volt2 # 电池电压[V]

        elif isinstance(tlm, StdTlm1) :
            self.BestGPS    = tlm.BestGPS
            self.GPS1SatCnt = tlm.GPS1SatNum
            self.GPS1Status = tlm.GPS1Status
            self.GPS1PDOP   = tlm.GPS1DOP * 0.01

            self.GPS2SatCnt = tlm.GPS2SatNum
            self.GPS2Status = tlm.GPS2Status
            self.GPS2PDOP   = tlm.GPS2DOP * 0.01

            self.GndSpeed   = tlm.GndSpd  * 0.01 # 地速:m/s
            self.GPSAlt     = tlm.GPSAlt  * 0.01 # 海拔:meter
            self.WindDir    = tlm.WindDir * 360.0 / 256.0 # 风向:0~360°
            self.WindSpd    = tlm.WindSpd * 0.01 # 风速:m/s

        elif isinstance(tlm, StdTlm2) :
            # 伺服输出, 范围 0~200，转换为 1000~2000 us
            self.Servo = [
                1000 + 5 * tlm.SRV1,
                1000 + 5 * tlm.SRV2,
                1000 + 5 * tlm.SRV3,
                1000 + 5 * tlm.SRV4,
                1000 + 5 * tlm.SRV5,
                1000 + 5 * tlm.SRV6,
                1000 + 5 * tlm.SRV7,
                1000 + 5 * tlm.SRV8
            ]

            # RC遥控器输出, 范围 -100~100
            self.RCRoll  = tlm.RCR
            self.RCPitch = tlm.RCP
            self.RCYaw   = tlm.RCY
            self.RCThr   = tlm.RCT

        elif isinstance(tlm, StdTlm3) :
            self.Yaw       = tlm.Yaw * 0.01
            self.NavYaw    = tlm.TargetYaw * 0.01
            self.Course    = tlm.Hdg * 0.01
            self.NavCourse = tlm.TargetHdg * 0.01
            self.NavAlt    = tlm.TargetAlt * 0.01
            self.Aileron   = tlm.Aileron
            self.Elevator  = tlm.Elevator
            self.Rudder    = tlm.Rudder
            self.Throttle  = tlm.Throttle
            self.CmdStep   = tlm.CmdStep

        elif isinstance(tlm, StdTlm4) :
            self.NavPitch   = tlm.TargetPitch * 0.01
            self.NavRoll    = tlm.TargetRoll * 0.01
            self.Alt        = tlm.Alt * 0.01
            self.AltDot     = tlm.AltDot * 0.01
            self.Temp       = tlm.Temp
            self.ClimbState = tlm.ClimbState
            self.Voyage     = tlm.Voyage # 总航程

            # 状态标记位
            self.IsReady    = bit_test(tlm.Flags, 0)
            self.IsOffGnd   = bit_test(tlm.Flags, 1)
            self.IsLocked   = bit_test(tlm.Flags, 2)
            self.IsVibeWarn = bit_test(tlm.Flags, 3)
            self.IsLogOk    = bit_test(tlm.Flags, 4)
            self.IsBatFail  = bit_test(tlm.Flags, 5)

        # debugging information
        #print (f'{self.Roll:.2f} {self.Pitch:.2f} {self.Yaw:.2f} {self.Lat:.6f} {self.Lng:.6f}')

    # 吊舱遥测信息更新处理
    def OnGimbalTlmRcvEvent(self, gimbal):
        if isinstance(gimbal, GimbalTlm) :
            self.GimbalRoll      = 0.01 * gimbal.Roll
            self.GimbalPitch     = 0.01 * gimbal.Pitch
            self.GimbalYaw       = 0.01 * gimbal.Yaw
            self.GimbalZoomLevel = 0.1  * gimbal.Zoom
            self.GimbalLaserDist = 0.01 * gimbal.Dist

