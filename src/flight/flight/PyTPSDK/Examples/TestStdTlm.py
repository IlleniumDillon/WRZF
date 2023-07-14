# 导入搜索路径 '../'
#------------------------------------------------------------------------------
import sys
import os

rootDir = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
print (rootDir)
sys.path.insert(0, rootDir)
#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
from TPLink import *
import time

# lnk = tplink('COM4')
# print (lnk.IsOpened())

# fmt='=hhiiHHBBi'
# print (f'TEST Size={struct.calcsize(fmt)}')

print (f'StdTlm0 Size={struct.calcsize(StdTlm0.fmt)}')
print (f'StdTlm1 Size={struct.calcsize(StdTlm1.fmt)}')
print (f'StdTlm2 Size={struct.calcsize(StdTlm2.fmt)}')
print (f'StdTlm3 Size={struct.calcsize(StdTlm3.fmt)}')
print (f'StdTlm4 Size={struct.calcsize(StdTlm4.fmt)}')

# SIZE_STD_TLM_0     28
# SIZE_STD_TLM_1     30
# SIZE_STD_TLM_2     28
# SIZE_STD_TLM_3     30
# SIZE_STD_TLM_4     30

# SIZE_HIL_TLM       14
# SIZE_BCST_TLM      22
# SIZE_GIMBAL_TLM    20
# SIZE_OBSTACLE_TLM  13

while True:
    time.sleep(1)
