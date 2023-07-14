import struct

# ref https://docs.python.org/zh-cn/3/library/struct.html#format-characters
"""
c char               1
b signed char        1
B unsigned char      1
h short              2
H unsigned short     2
i int                4
I unsigned int       4
l long               4
L unsigned long      4
q long long          8
Q unsigned long long 8
f float              4
d double             8
s char[]
"""

class StdTlm0(object):
    fmt='=hhiiHHBBiHHH'
    def __init__(self):
        self.Pitch      = 0 # Current pitch angle         ,cd
        self.Roll       = 0 # Current roll angle          ,cd
        self.Lng        = 0 # Current Longitude           ,1e7*deg
        self.Lat        = 0 # Current Latitude            ,1e7*deg
        self.Speed      = 0 # Current speed               ,cm/s
        self.TargetSpd  = 0 # Target speed                ,cm/s
        self.FlightMode = 0 # Current Flight Mode         ,
        self.RCState    = 0 # 0-OK,1-LostFrame,2-FailSafe,3-NoRC,
        self.BaroAlt    = 0 # Barometer Altitude          ,cm
        self.AGL        = 0 # Altitude Above Ground Level ,cm
        self.Volt1      = 0 # Battery1 voltage            ,0.01V
        self.Volt2      = 0 # Battery2 voltage            ,0.01V

    # 将二进制格式反序列化为对象
    def Unserialize(self, data):
        ret = struct.unpack(self.fmt, data)
        self.Pitch      = ret[0]
        self.Roll       = ret[1]
        self.Lng        = ret[2]
        self.Lat        = ret[3]
        self.Speed      = ret[4]
        self.TargetSpd  = ret[5]
        self.FlightMode = ret[6]
        self.RCState    = ret[7]
        self.BaroAlt    = ret[8]
        self.AGL        = ret[9]
        self.Volt1      = ret[10]
        self.Volt2      = ret[11]

class StdTlm1(object):
    fmt='=hhiiHBBHiBBHBBH'
    def __init__(self):
        self.Pitch      = 0 # Current pitch angle         ,cd
        self.Roll       = 0 # Current roll angle          ,cd
        self.Lng        = 0 # Current Longitude           ,1e7*deg
        self.Lat        = 0 # Current Latitude            ,1e7*deg
        self.WindSpd    = 0 # Wind horizontal speed       ,cm/s
        self.WindDir    = 0 # Wind horizon direction      ,1.41deg
        self.BestGPS    = 0 # The best GNSS receiver ID   ,
        self.GndSpd     = 0 # Ground horizon speed        ,cm/s
        self.GPSAlt     = 0 # GNSS Altitude               ,cm
        self.GPS1SatNum = 0 # GPS1 Numbers of satellite   ,
        self.GPS1Status = 0 # GPS1 Status code of receiver,
        self.GPS1DOP    = 0 # GPS1 100 x PDOP             ,x100
        self.GPS2SatNum = 0 # GPS2 Numbers of satellite   ,
        self.GPS2Status = 0 # GPS2 Status code of receiver,
        self.GPS2DOP    = 0 # GPS2 100 x PDOP             ,x100

    # 将二进制格式反序列化为对象
    def Unserialize(self, data):
        ret = struct.unpack(self.fmt, data)
        self.Pitch      = ret[0]
        self.Roll       = ret[1]
        self.Lng        = ret[2]
        self.Lat        = ret[3]
        self.WindSpd    = ret[4]
        self.WindDir    = ret[5]
        self.BestGPS    = ret[6]
        self.GndSpd     = ret[7]
        self.GPSAlt     = ret[8]
        self.GPS1SatNum = ret[9]
        self.GPS1Status = ret[10]
        self.GPS1DOP    = ret[11]
        self.GPS2SatNum = ret[12]
        self.GPS2Status = ret[13]
        self.GPS2DOP    = ret[14]
class StdTlm2(object):
    fmt='hhiiBBBBBBBBbbbbHH'
    def __init__(self):
        self.Pitch = 0 # Current pitch angle         ,cd          */
        self.Roll  = 0 # Current roll angle          ,cd          */
        self.Lng   = 0 # Current Longitude           ,1e7*deg     */
        self.Lat   = 0 # Current Latitude            ,1e7*deg     */
        self.SRV1  = 0 # Servo channel1 value        ,0~200       */
        self.SRV2  = 0 # Servo channel2 value        ,0~200       */
        self.SRV3  = 0 # Servo channel3 value        ,0~200       */
        self.SRV4  = 0 # Servo channel4 value        ,0~200       */
        self.SRV5  = 0 # Servo channel5 value        ,0~200       */
        self.SRV6  = 0 # Servo channel6 value        ,0~200       */
        self.SRV7  = 0 # Servo channel7 value        ,0~200       */
        self.SRV8  = 0 # Servo channel8 value        ,0~200       */
        self.RCR   = 0 # RC input of Roll            ,-100~100    */
        self.RCP   = 0 # RC input of Pitch           ,-100~100    */
        self.RCY   = 0 # RC input of Yaw             ,-100~100    */
        self.RCT   = 0 # RC input of Throttle        ,-100~100    */
        self.Cur1  = 0 # Battery1 current            ,0.01A       */
        self.Cur2  = 0 # Battery2 current            ,0.01A       */

    # 将二进制格式反序列化为对象
    def Unserialize(self, data):
        ret = struct.unpack(self.fmt, data)
        self.Pitch = ret[0]
        self.Roll  = ret[1]
        self.Lng   = ret[2]
        self.Lat   = ret[3]
        self.SRV1  = ret[4]
        self.SRV2  = ret[5]
        self.SRV3  = ret[6]
        self.SRV4  = ret[7]
        self.SRV5  = ret[8]
        self.SRV6  = ret[9]
        self.SRV7  = ret[10]
        self.SRV8  = ret[11]
        self.RCR   = ret[12]
        self.RCP   = ret[13]
        self.RCY   = ret[14]
        self.RCT   = ret[15]
        self.Cur1  = ret[16]
        self.Cur2  = ret[17]

class StdTlm3(object):
    fmt='=hhiiHHHHibbbBH'
    def __init__(self):
        self.Pitch     = 0 # Current pitch angle         ,cd          */
        self.Roll      = 0 # Current roll angle          ,cd          */
        self.Lng       = 0 # Current Longitude           ,1e7*deg     */
        self.Lat       = 0 # Current Latitude            ,1e7*deg     */
        self.Yaw       = 0 # Current yaw                 ,cd          */
        self.Hdg       = 0 # The direction of movement   ,cd          */
        self.TargetYaw = 0 # The target yaw angle        ,cd          */
        self.TargetHdg = 0 # The target heading angle    ,cd          */
        self.TargetAlt = 0 # The final target altitude   ,cm          */
        self.Aileron   = 0 # Actuator of Aileron         ,-100~100    */
        self.Elevator  = 0 # Actuator of Elevator        ,-100~100    */
        self.Rudder    = 0 # Actuator of Rudder          ,-100~100    */
        self.Throttle  = 0 # Actuator of Throttle        ,0~100       */
        self.CmdStep   = 0 # Mission Step in running     ,            */

    # 将二进制格式反序列化为对象
    def Unserialize(self, data):
        ret = struct.unpack(self.fmt, data)
        self.Pitch     = ret[0]
        self.Roll      = ret[1]
        self.Lng       = ret[2]
        self.Lat       = ret[3]
        self.Yaw       = ret[4]
        self.Hdg       = ret[5]
        self.TargetYaw = ret[6]
        self.TargetHdg = ret[7]
        self.TargetAlt = ret[8]
        self.Aileron   = ret[9]
        self.Elevator  = ret[10]
        self.Rudder    = ret[11]
        self.Throttle  = ret[12]
        self.CmdStep   = ret[13]

class StdTlm4(object):
    fmt='=hhiihhbBHihI'
    def __init__(self):
        self.Pitch       = 0 # Current pitch angle         ,cd          */
        self.Roll        = 0 # Current roll angle          ,cd          */
        self.Lng         = 0 # Current Longitude           ,1e7*deg     */
        self.Lat         = 0 # Current Latitude            ,1e7*deg     */
        self.TargetPitch = 0 # The target pitch angle      ,cd          */
        self.TargetRoll  = 0 # The target roll  angle      ,cd          */
        self.Temp        = 0 # Temperature of device       ,Celsius     */
        self.ClimbState  = 0 # 0-Level,1-Climb,2-Descend   ,            */
        self.Flags       = 0 # Status bit flags            ,            */
        self.Alt         = 0 # Altitude from home location ,cm          */
        self.AltDot      = 0 # Change rate of altitude     ,cm/s        */
        self.Voyage      = 0 # Total voyage in meter       ,meter       */

    # 将二进制格式反序列化为对象
    def Unserialize(self, data):
        ret = struct.unpack(self.fmt, data)
        self.Pitch      = ret[0]
        self.Roll       = ret[1]
        self.Lng        = ret[2]
        self.Lat        = ret[3]
        self.TargetPitch= ret[4]
        self.TargetRoll = ret[5]
        self.Temp       = ret[6]
        self.ClimbState = ret[7]
        self.Flags      = ret[8]
        self.Alt        = ret[9]
        self.AltDot     = ret[10]
        self.Voyage     = ret[11]

class GimbalTlm(object):
    fmt='=iifhhhH'
    def __init__(self):
        self.Lng   = 0 # Trace Object latitude       ,1e7*deg
        self.Lat   = 0 # Trace Object longitude      ,1e7*deg
        self.Dist  = 0 # Range finder distance       ,m
        self.Pitch = 0 # Gimbal ENC pitch angle      ,cd
        self.Roll  = 0 # Gimbal ENC roll angle       ,cd
        self.Yaw   = 0 # Gimbal ENC yaw angle        ,cd
        self.Zoom  = 0 # Zoom Level                  ,0.1

    # 将二进制格式反序列化为对象
    def Unserialize(self, data):
        ret = struct.unpack(self.fmt, data)
        self.Lng   = ret[0]
        self.Lat   = ret[1]
        self.Dist  = ret[2]
        self.Pitch = ret[3]
        self.Roll  = ret[4]
        self.Yaw   = ret[5]
        self.Zoom  = ret[6]
