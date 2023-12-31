'''
FilePath: fsm.py
Author: Ballade-F     258300018@qq.com
Date: 2023-07-14 11:20:55
LastEditors: Please set LastEditors
LastEditTime: 2023-07-25 21:07:17
Copyright: 2023  All Rights Reserved.
Descripttion: 
'''

class FSM:
    def __init__(self):
        self.current_state = 'power_up'
        self.last_state = 'power_up'

    def transition(self,event):
        _transitions = {
            #state_from       event         state_to
            'power_up' : {'takeoff' : 'go_start'},
            'go_up_height' : {'rightHeight' : 'go_start'},
            # 'go_start' : {'catchCar' : 'follow_number_high'},
            'go_start' : {'catchCar' : 'follow_number'},
            'follow_number' : {'changeCar' : 'follow_number_high'},
            'follow_number_high' : {'rightHeight' : 'follow_point',
                                    'rightPix' : 'follow_number'},
            'follow_point' : {'rightPos' : 'follow_number_high'}, 
        }

        if event in _transitions[self.current_state]:
            self.last_state = self.current_state
            self.current_state = _transitions[self.current_state][event]
            print("new state:"+self.current_state)


    def getState(self):
        return self.current_state
    
    def getLastState(self):
        return self.last_state