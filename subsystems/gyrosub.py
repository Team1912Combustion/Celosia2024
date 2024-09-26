from constants import GyroConstants as Gc
from commands2 import Subsystem
from wpilib import SmartDashboard
from navx import AHRS

class GyroSub(Subsystem):
    def __init__(self):
        self.__g = AHRS.create_spi(Gc.PORT, Gc.BITRATE, Gc.UPDATE_HZ)
        self.reset()

    def periodic(self):
        # native LabVIEW Dashboard integration
        SmartDashboard.putNumber("Gyro", self.getRot(True))

    def getRot(self, asDeg: bool = False):
        # overhead...?
        rot = self.__g.getRotation2d()
        return rot.degrees() if asDeg else rot
    
    def getRate(self): return self.__g.getRate()
    
    def reset(self): self.__g.reset()