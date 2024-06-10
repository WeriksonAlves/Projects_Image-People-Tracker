import os
from typing import Union
import numpy as np

class ServoPositionSystem:
    def __init__(self, number_servo: int = 0):
        self.number_servo = number_servo
        if self.number_servo == 0:
            self.enable = False
        else:
            self.enable = True
    
    def action_servo(self, center_person: bool, dist_center_person: tuple):
        if self.enable:
            if center_person:
                self.move_servo(0)
            else:
                if dist_center_person[0] < 0:
                    self.move_servo(1)
                else:
                    self.move_servo(-1)

