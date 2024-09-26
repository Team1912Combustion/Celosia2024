from commands2.button import CommandXboxController
from commands2 import Subsystem, Command, cmd
from constants import FeederConstants as Fc
from wpilib import Spark

class FeedSub(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.__m = Spark(Fc.ID)

    def setSpeed(self, speed: float) -> Command:
        return cmd.runOnce(lambda: self.__m.set(speed), self)
    
    def bind(self, controller: CommandXboxController):
        controller.a() \
            .onTrue(self.setSpeed(1.0)) \
            .onFalse(self.setSpeed(0.0))
        return self