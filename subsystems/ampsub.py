from commands2.button import CommandXboxController
from commands2 import Subsystem, Command, cmd
from constants import AmpConstants as Ac
from wpilib import PWMVictorSPX

class AmpSub(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.__m = PWMVictorSPX(Ac.MOTOR_ID)

    def setSpeed(self, speed: float) -> Command:
        return cmd.runOnce(lambda: self.__m.set(speed), self)
    
    def bind(self, controller: CommandXboxController):
        controller.povDown() \
            .onTrue(self.setSpeed(-0.1)) \
            .onFalse(self.setSpeed(0.0))
        controller.povUp() \
            .onTrue(self.setSpeed(0.33)) \
            .onFalse(self.setSpeed(0.0))
        return self