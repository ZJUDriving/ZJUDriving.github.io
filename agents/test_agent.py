import carla
import numpy as np

class testAgent(object):
    """ basic ad agent"""
    def __init__(self) -> None:
        super().__init__()
        self.destination = None
        self.trajectory = None
        self.state = None
    
    def take_action(self, state):
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.5
        control.hand_brake = False
        control.manual_gear_shift = False
        return control