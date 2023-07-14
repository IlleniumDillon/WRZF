# 导入搜索路径 '../'
#------------------------------------------------------------------------------
import sys
import os
import time

rootDir = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
print (rootDir)
sys.path.insert(0, rootDir)
#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
from UAVItem import *
from LinkDef import *
#------------------------------------------------------------------------------

uav = uavitem('COM4')
print (f'IsConnected={uav.IsConnected()}')

uav.Link.DebugEn = True

while True:
    # time.sleep(1)
    # print (f'{uav.Pitch:.2f}')
    cmd = input()
    print (f'CMD> {cmd}')
    if cmd == 'takeoff':
        uav.Link.SendCmdTakeoff()
    if cmd == 'rtl':
        uav.Link.SendCmdSetFlightMode(flightmode.AUTO, flightstage.RTL)
    if cmd == 'hover 1':
        uav.Link.SendCmdEnterHover()
    if cmd == 'hover 0':
        uav.Link.SendCmdExitHover()
    if cmd == 'offboard 1':
        uav.Link.SendCmdOffboardEnter()
    if cmd == 'offboard 0':
        uav.Link.SendCmdOffboardLeave()

    if cmd == 'go pos':
        while True:
            uav.Link.SendCmdOffboardSetPos(offboardCoord.NEU, 1000, 10.123, 10.345, 30.456, 0)
            time.sleep(0.5)

    if cmd == 'go vel':
        while True:
            uav.Link.SendCmdOffboardSetVel(offboardCoord.NEU, 1000, 3.4, 2.5, 0.2, 0)
            time.sleep(0.5)

    if cmd == 'go lla':
        while True:
            uav.Link.SendCmdOffboardSetPos(offboardCoord.WGS84, 1000, 41.6649831*1e7, 123.6705423*1e7, 200.12, 0)
            time.sleep(0.5)
