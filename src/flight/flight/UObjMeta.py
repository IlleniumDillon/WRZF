from enum import IntEnum

#----------------------------------------------------------------
# 元数据类型编码
#----------------------------------------------------------------
class metadatatype(IntEnum):
    DT_S8   = 0x01,  #  8-bit   signed integer
    DT_U8   = 0x00,  #  8-bit unsigned integer
    DT_U16  = 0x02,  # 16-bit unsigned integer
    DT_S16  = 0x03,  # 16-bit   signed integer
    DT_U32  = 0x04,  # 32-bit unsigned integer
    DT_S32  = 0x05,  # 32-bit   signed integer
    DT_FP32 = 0x06   # 32-bit floating point

#----------------------------------------------------------------
# UObject Meta结构，用于描述对象结构
#----------------------------------------------------------------
class uobjmeta(object):
    def __init__(self):
        self.TypeName   = ''    # 类型名称
        self.TypeId     = 0     # 类型ID
        self.TypeSz     = 0     # 对象数据大小
        self.InstCnt    = 0     # 实例个数
        self.IsSetting  = False # 是否为配置项
        self.MemCnt     = 0     # 成员变量个数
        self.MemName    = []    # 成员变量名称
        self.MemUnit    = []    # 成员变量单位
        self.MemType    = []    # 成员变量类型
        self.MemSize    = []    # 成员变量大小

