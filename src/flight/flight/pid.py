'''
FilePath: pid.py
Author: Ballade-F     258300018@qq.com
Date: 2023-07-14 11:20:37
LastEditors: Please set LastEditors
LastEditTime: 2023-07-14 11:20:44
Copyright: 2023  All Rights Reserved.
Descripttion: 
'''

class PID:
    def __init__(self,pid_p,pid_i,pid_d,sum_max,out_max,out_min,T) :
        self.p=pid_p
        self.i=pid_i
        self.d=pid_d

        self.T = T

        self.error=0
        self.error_last=0
        self.error_sum=0
        self.sum_max=sum_max

        self.out=0
        self.out_max=out_max
        self.out_min=out_min

    def pidClear(self):
        self.error = 0
        self.error_last=0
        self.error_sum=0
        self.out=0

    def pidUpdate(self,des,current):
        self.error = des - current
        self.out = self.p*self.error + self.T*self.i*self.error_sum + (1/self.T)*self.d*(self.error-self.error_last)

        if self.out > self.out_max:
            self.out = self.out_max
        elif self.out < self.out_min:
            self.out = self.out_min
        else:
            pass#异常
        
        self.error_last = self.error
        self.error_sum += self.error

        if self.error_sum > self.sum_max:
            self.error_sum = self.sum_max

