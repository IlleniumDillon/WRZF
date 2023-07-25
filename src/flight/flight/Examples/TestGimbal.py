# 导入搜索路径 '../'
#------------------------------------------------------------------------------
import sys
import os
import time

rootDir = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
print (rootDir)
sys.path.insert(0, rootDir)


#------------------------------------------------------------------------------
from UAVItem import *
from LinkDef import *
#------------------------------------------------------------------------------
from TPLink import *
import time

uav = uavitem('COM4')
is_connected = uav.IsConnected()
print (f'IsConnected={is_connected}')

while is_connected:
    cmd = input()
    print (f'CMD> {cmd}')
    if cmd == 'left':
        uav.Link.SendCmdPodMoveLeft()
    if cmd == 'right':
        uav.Link.SendCmdPodMoveRight()
    if cmd == 'down':
        uav.Link.SendCmdPodMoveDown()
    if cmd == 'up':
        uav.Link.SendCmdPodMoveUp()
    if cmd == 'stop':
        uav.Link.SendCmdPodMoveStop()
    if cmd == 'zero':
        uav.Link.SendCmdPodMoveZero()
    if cmd == 'track 1':
        uav.Link.SendCmdPodTraceCtrl(True, 0.6, 0.6)
    if cmd == 'track 0':
        uav.Link.SendCmdPodTraceCtrl(False, 0, 0)
