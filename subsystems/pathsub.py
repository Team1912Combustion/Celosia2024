from wpimath.controller import RamseteController, SimpleMotorFeedforwardMeters
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint
from wpimath.geometry import Pose2d
from wpimath.trajectory import *
from commands2 import Subsystem
from constants import AutoConstants as Ac

class PathSub(Subsystem):
    def __init__(self):
        self.ram = RamseteController(Ac.kRamseteB, Ac.kRamseteZeta)

        self.feedforward = SimpleMotorFeedforwardMeters(
                kS=Ac.ksVolts,
                kV=Ac.kvVoltSecondsPerMeter,
                kA=Ac.kaVoltSecondsSquaredPerMeter)

        self.voltConstraint = DifferentialDriveVoltageConstraint(
            self.feedforward,
            Ac.kDriveKinematics,
            maxVoltage=10)

        self.trajCfg = TrajectoryConfig(
            Ac.kMaxSpeedMetersPerSecond,
            Ac.kMaxAccelerationMetersPerSecondSquared)

        self.trajCfg.setKinematics(Ac.kDriveKinematics)
        self.trajCfg.addConstraint(self.voltConstraint)

    def loadJson(name):
        pass