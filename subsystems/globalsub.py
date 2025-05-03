from wpilib.cameraserver import *
from wpilib import DriverStation
from commands2 import Subsystem
from typing import Callable
from wpilib.event import *

class GlobalSub(Subsystem):
    def __init__(self):
        self.__l = EventLoop()
        CameraServer.launch()
        
        DriverStation.silenceJoystickConnectionWarning(True)