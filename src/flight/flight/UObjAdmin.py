from UDevType import *

#----------------------------------------------------------------
# UAV Object对象管理器
#----------------------------------------------------------------
class uobjadmin(object):
    def __init__(self):
        self.Signature  = 0                     # UObject Meta签名
        self.DevType    = udevtype.UNKNOWN      # 结构对象大小，单位字节
