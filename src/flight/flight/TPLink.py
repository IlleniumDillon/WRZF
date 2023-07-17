import serial
import threading
import flight.CRC16
import logging
import time

from flight.LinkDef import *
from flight.Telemetry import *
from flight.UAVItem import *

#----------------------------------------------------------------
class tplink(object):
    def __init__(self, port_num, owner):
        self.__rcvBuf  = bytearray(512)  # 数据接收缓冲区
        self.__rcvLen  = 0               # TPLINK对象名称
        self.__comm    = None            # 当前关联通讯接口
        self.__rcvTask = None            # 通讯接口接收线程
        self.__running = False
        self.__allocPacketSn = 0
        self.__locker  = threading.Lock()  # 发送信息时的线程锁

        self.Name      = ''              # TPLINK对象名称
        self.DevAddr   = 0xFF            # Link对象关联的设备ID(默认广播地址)
        self.Owner     = owner
        self.DebugEn   = False
        self.Open(port_num)

    def __putchar(self, c):
        # 过滤无效帧头数据
        if 0 == self.__rcvLen and linkdef.Header1 != c:
            return
        if 1 == self.__rcvLen and linkdef.Header2 != c:
            self.__rcvLen = 0
            return

        self.__rcvBuf[self.__rcvLen] = c
        self.__rcvLen += 1

        if self.__rcvLen < linkdef.MinPkSize:
            return

        if self.__rcvLen > linkdef.MaxPkSize:
            self.__rcvLen = 0
            # Overflow
            return

        if self.__rcvLen == self.__rcvBuf[2]:
            self.__analyze(self.__rcvBuf, self.__rcvLen)
            self.__rcvLen = 0 # For next packet

        return

    def __can_accept(self, data, len):
        if data[0] != linkdef.Header1 or data[1] != linkdef.Header2:
            return False
        # print (' '.join(format(data[i], '02x') for i in range(len)))
        # 计算并验证校验和
        exp = data[len - 2] + data[len - 1]*256
        ret = CRC16.Calc(data[2:len-2])
        return ret == exp

    def __proc_response(self, data, len, offset):
        pass

    def __proc_telemetry(self, data):
        # print (f'TLM type: {data[0]}')
        if data[0] <= tlmtype.STD4:
            # 标准遥测处理
            stdtlm = None
            if   tlmtype.STD0 == data[0]:
                stdtlm = StdTlm0()
            elif tlmtype.STD1 == data[0]:
                stdtlm = StdTlm1()
            elif tlmtype.STD2 == data[0]:
                stdtlm = StdTlm2()
            elif tlmtype.STD3 == data[0]:
                stdtlm = StdTlm3()
            elif tlmtype.STD4 == data[0]:
                stdtlm = StdTlm4()

            if self.Owner is not None and stdtlm is not None:
                stdtlm.Unserialize(data[1:])
                self.Owner.OnStdTlmRcvEvent(stdtlm)

        if tlmtype.GIMBAL == data[0]:
            gtlm = GimbalTlm()
            gtlm.Unserialize(data[1:])
            if self.Owner is not None:
                self.Owner.OnGimbalTlmRcvEvent(gtlm)

    # 对接收到的完整报文进行解析处理
    def __analyze(self, data, len):
        if False == self.__can_accept(data, len):
            return
        src_addr = data[3]
        dst_addr = data[4]
        sn       = data[5]

        if 0 == sn:
            # Telemetry Packets
            self.__proc_telemetry(data[6:len-2])
            pass
        elif 0xff == sn:
            # 心跳帧 [建立设备地址表]
            pass
        else:
            # Response Packets
            self.__proc_response(data, len, 6)

    def __proc_data_received(self):
        while self.__running:
            data = self.__comm.read_all()
            if len(data) == 0:
                time.sleep(0.01)
                continue
            for c in data:
                self.__putchar(c)

    def __gen_send_data(self, payload, len, sn):
        i = 0
        snd_buf    = bytearray(len + 8)
        snd_buf[i] = linkdef.Header1; i += 1
        snd_buf[i] = linkdef.Header2; i += 1
        snd_buf[i] = len + 8;         i += 1 # 报文总长度
        snd_buf[i] = 0x00;            i += 1 # 报文源地址【地面站地址，默认为0】
        snd_buf[i] = self.DevAddr;    i += 1 # 报文目的地址
        snd_buf[i] = sn;              i += 1 # 报文序号

        for j in range(len): snd_buf[i] = payload[j]; i += 1

        crc = CRC16.Calc(snd_buf[2:i])       # 计算CRC校验码
        snd_buf[i] = crc & 0xFF;      i += 1 # CRC低字节
        snd_buf[i] = (crc>>8) & 0xFF; i += 1 # CRC高字节

        return snd_buf

    # 分配报文序列号（不需要响应帧的SN固定为0xFF）
    def __alloc_packet_sn(self, needRsp):
        if False == needRsp:
            return 0xFF

        # 序列号0x00 预留给遥测报文
        if 0 == self._allocPacketSn:
            self._allocPacketSn = 1

        # 序列号0xFF 预留给心跳报文和其他不需要响应的报文
        if 0xFF == self.__allocPacketSn:
            self.__allocPacketSn = 1

        # 如果该报文等待响应，记录待响应报文序列号
        # _waitPacketSn = self.__allocPacketSn
        return self.__allocPacketSn

    def __safe_write_payload(self, payload, len):
        self.__locker.acquire()
        snd_buf = self.__gen_send_data(payload, len, self.__alloc_packet_sn(False))
        self.__comm.write(snd_buf)
        self.__locker.release()
        if self.DebugEn:
            print ('SND:' + ' '.join(format(x, '02X') for x in snd_buf))

#----------------------------------------------------------------
# 公共成员函数集合
#----------------------------------------------------------------
    def Open(self, port_name):
        try:
            self.__comm = serial.Serial(
                port=port_name,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE)
            ret = self.__comm.is_open
        except  serial.SerialException as e:
            print (e)
            ret = False

        if ret :
            self.__running = True
            self.__rcvTask = threading.Thread(target=self.__proc_data_received, daemon = True)
            self.__rcvTask.name = f'{self.Name}RcvThread'
            self.__rcvTask.start()

        return ret

    def Close(self):
        if self.__comm is None:
            return
        self.__running = False
        self.__comm.close()

    #----------------------------------------------------------------
    # Link是否打开，可进行正常收发
    #----------------------------------------------------------------
    def IsOpened(self):
        if self.__comm is None:
            return False
        return self.__comm.is_open

    #----------------------------------------------------------------
    # 发送飞行模式设置指令
    #----------------------------------------------------------------
    def SendCmdSetFlightMode(self, fm, fs):
        if False == self.IsOpened():
            return
        i = 0
        payload    = bytearray(linkdef.MaxPkSize)
        payload[i] = linkcmd.SET_FLIGHT_MODE;   i += 1
        payload[i] = (fs << 4 | fm);            i += 1

        self.__safe_write_payload(payload, i)
        logging.debug('SendCmdSetFlightMode')

    #----------------------------------------------------------------
    # 发送 Offboard 命令请求指令 [进入Offboard模式]
    #----------------------------------------------------------------
    def SendCmdOffboardEnter(self):
        if False == self.IsOpened():
            return
        i = 0
        payload = bytearray(linkdef.MaxPkSize)
        payload[i] = linkcmd.OFFBOARD_REQ;  i += 1
        payload[i] = 0x04;                  i += 1

        self.__safe_write_payload(payload, i)
        logging.debug('SendCmdOffboardEnter')

    #----------------------------------------------------------------
    # 发送 Offboard 命令请求指令 【异步模式|离开Offboard模式】
    #----------------------------------------------------------------
    def SendCmdOffboardLeave(self):
        if False == self.IsOpened():
            return

        i = 0
        payload = bytearray(linkdef.MaxPkSize)
        payload[i] = linkcmd.OFFBOARD_REQ; i += 1
        payload[i] = 0x05;                 i += 1

        self.__safe_write_payload(payload, i)
        logging.debug('SendCmdOffboardLeave')

    #----------------------------------------------------------------
    # 发送 Offboard 命令请求指令 [设置目标位置]
    #----------------------------------------------------------------
    def SendCmdOffboardSetPos(self, coord, duration, posX, posY, posZ, deltaYaw):
        if False == self.IsOpened():
            return

        i = 0
        payload = bytearray(linkdef.MaxPkSize)
        payload[i] = linkcmd.OFFBOARD_REQ;   i+=1
        payload[i] = offboardReq.SET_POS;    i+=1 # 设置目标POS
        payload[i] = coord;                  i+=1 # 坐标系类型
        payload[i] = duration & 0xFF;        i+=1
        payload[i] = duration >>8;           i+=1

        if coord != offboardCoord.WGS84:
            for b in struct.pack('f', posX):   payload[i] = b; i+=1
            for b in struct.pack('f', posY):   payload[i] = b; i+=1
        else:
            for b in int.to_bytes(int(posX),4, 'little'):payload[i] = b; i+=1
            for b in int.to_bytes(int(posY),4, 'little'):payload[i] = b; i+=1

        for b in struct.pack('f', posZ):       payload[i] = b; i+=1
        for b in struct.pack('f', deltaYaw):   payload[i] = b; i+=1

        self.__safe_write_payload(payload, i)
        logging.debug('SendCmdOffboardSetPos')

    #----------------------------------------------------------------
    # 发送 Offboard 命令请求指令 [设置目标速度]
    #----------------------------------------------------------------
    def SendCmdOffboardSetVel(self, coord, duration, velX, velY, velZ, deltaYaw):
        if False == self.IsOpened():
            return

        i = 0
        payload = bytearray(linkdef.MaxPkSize)
        payload[i] = linkcmd.OFFBOARD_REQ; i+=1
        payload[i] = offboardReq.SET_VEL;  i+=1 # 设置目标速度
        payload[i] = coord;                i+=1 # 坐标系类型
        payload[i] = duration & 0xFF;      i+=1
        payload[i] = duration >>8;         i+=1

        for b in struct.pack('f', velX):     payload[i] = b; i+=1
        for b in struct.pack('f', velY):     payload[i] = b; i+=1
        for b in struct.pack('f', velZ):     payload[i] = b; i+=1
        for b in struct.pack('f', deltaYaw): payload[i] = b; i+=1

        self.__safe_write_payload(payload, i)
        logging.debug('SendCmdOffboardSetVel')

    #----------------------------------------------------------------
    # 向飞控发送RTCM数据，支持RTK功能
    #----------------------------------------------------------------
    def SendCmdRtcmUpdate(self, data):
        if False == self.IsOpened():
            return
        pass

    #----------------------------------------------------------------
    # 发送DoAction指令
    # <param name="subCmd">子级命令编码</param>
    # <param name="para">命令参数</param>
    # <param name="needRsp">是否需要反馈信息</param>
    #----------------------------------------------------------------
    def SendCmdDoAction(self, subCmd, para):
        if False == self.IsOpened():
            return

        i = 0
        payload = bytearray(linkdef.MaxPkSize)
        payload[i] = linkcmd.DO_ACTION;         i += 1 #
        payload[i] = subCmd;                    i += 1 # 子级命令编码
        if para is not None:
            for b in para : payload[i] = b;     i+=1
        self.__safe_write_payload(payload, i)

    #----------------------------------------------------------------
    # DoAction指令工具函数-进入Hover模式
    #----------------------------------------------------------------
    def SendCmdEnterHover(self):
        if False == self.IsOpened():
            return

        self.SendCmdDoAction(actionsubcmd.HOVER, [1])
        logging.debug('SendCmdEnterHover')

    #----------------------------------------------------------------
    # DoAction指令工具函数-离开Hover模式
    #----------------------------------------------------------------
    def SendCmdExitHover(self):
        if False == self.IsOpened():
            return

        self.SendCmdDoAction(actionsubcmd.HOVER, [0])
        logging.debug('SendCmdExitHover')

    #----------------------------------------------------------------
    # DoAction指令工具函数-自主起飞
    #----------------------------------------------------------------
    def SendCmdTakeoff(self):
        if False == self.IsOpened():
            return

        self.SendCmdDoAction(actionsubcmd.TAKEOFF, None)
        logging.debug('SendCmdTakeoff')
