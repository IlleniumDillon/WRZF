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
from TPLink import *
import time

lnk = tplink('COM4', None)
print (lnk.IsOpened())

while True:
    time.sleep(1)

