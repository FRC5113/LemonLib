from ..control import LemonInput
from ..smart import SmartPreference

class Funnycontrollers:
    switch_time = SmartPreference(2.0)

    def __init__(self,port1: int, port2: int):
        self.con1 = LemonInput(0)
        self.con2 = LemonInput(1)
        self.last_time = 0.0
    
    def get_controller1(self) -> LemonInput:
        return self.con1
    
    def get_controller2(self) -> LemonInput:
        return self.con2
    
    def get_leftX_avg(self) -> float:
        return (self.con1.getLeftX() + self.con2.getLeftX()) / 2
    
    def get_leftY_avg(self) -> float:
        return (self.con1.getLeftY() + self.con2.getLeftY()) / 2
    
    def get_rightX_avg(self) -> float:
        return (self.con1.getRightX() + self.con2.getRightX()) / 2
    
    def get_rightY_avg(self) -> float:
        return (self.con1.getRightY() + self.con2.getRightY()) / 2
    
    def get_triggerL_avg(self) -> float:
        return (self.con1.getLeftTriggerAxis() + self.con2.getLeftTriggerAxis()) / 2  
    
    def get_triggerR_avg(self) -> float:
        return (self.con1.getRightTriggerAxis() + self.con2.getRightTriggerAxis()) / 2
    
    def get_switched_controller(self,time) -> LemonInput:
        current_time = time
        if current_time - self.last_time > self.switch_time:
            self.last_time = current_time
            return self.con2
        return self.con1
    
