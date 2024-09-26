import math
import numpy
import commands2

from commands.ramtest import RamTest
import constants

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.opticalsubsystem import OpticalSubsystem
from subsystems.limesubsystem import LimeSubsystem
from subsystems.sewsubsystem import SewSubsystem

from subsystems.ampsub import *
from subsystems.gyrosub import *
from subsystems.feedsub import *

from wpilib.cameraserver import CameraServer

from commands2.button import CommandXboxController

from commands2 import Command, SequentialCommandGroup, WaitCommand, ParallelCommandGroup

class RobotContainer:
    driverController = CommandXboxController(0)
    opsController = CommandXboxController(1)

    def __init__(self):
        """The container for the robot. Contains subsystems, OI devices, and commands."""
        self.optics = OpticalSubsystem()
        self.lime = LimeSubsystem()
        self.drive = DriveSubsystem(self.optics, self.lime, GyroSub())
        self.feeder = FeedSub().bind(self.opsController)
        self.intake = IntakeSubsystem()
        self.shooter = ShooterSubsystem()
        self.sew = SewSubsystem()
        self.amp = AmpSub().bind(self.opsController)

        CameraServer.launch()

        self.configureBindings()

        self.drive.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.drive.arcade(
                    -self.driverController.getLeftY(),
                    -self.driverController.getRightX(),
                ),
                self.drive,
            )
        )

    def configureBindings(self) -> None:
        self.opsController.b() \
            .onTrue(self.intake.setSpeed(0.5)) \
            .onFalse(self.intake.setSpeed(0.0))

        self.opsController.x() \
            .onTrue(self.shooter.setSpeed(1.0)) \
            .onFalse(self.shooter.setSpeed(0.0))

        self.opsController.y() \
            .onTrue(self.setAll(-0.5)) \
            .onFalse(self.setAll(0.0))

        self.opsController.rightBumper() \
            .onTrue(SequentialCommandGroup(
                self.intake.setSpeed(-0.5),
                WaitCommand(0.5),
                self.intake.setSpeed(0.0)))

        self.driverController.leftBumper().onTrue(self.drive.toggleRewindTime())

    def setAll(self, speed: float):
        return ParallelCommandGroup(
            self.intake.setSpeed(speed),
            self.feeder.setSpeed(speed),
            self.shooter.setSpeed(speed),
            self.amp.setSpeed(speed))

    def getAutonomousCommand(self) -> Command:

        return SequentialCommandGroup(
            self.shooter.setSpeed(1.0),
            self.feeder.setSpeed(1.0),
            WaitCommand(6), self.intake.setSpeed(0.5),
            WaitCommand(1), self.setAll(0.0),
            RamTest(self.drive, self.sew), self.drive.stopIt())

    #def cleanup(self): self.lime.disconnect()