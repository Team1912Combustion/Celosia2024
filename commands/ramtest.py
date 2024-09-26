from commands2 import SequentialCommandGroup
from wpimath.controller import PIDController
from custom.ramsetecommand import RamseteCommand
from subsystems.drivesubsystem import DriveSubsystem
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from subsystems.sewsubsystem import SewSubsystem
from constants import AutoConstants as Ac
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.geometry import Rotation2d

class RamTest(SequentialCommandGroup):
    def __init__(self, drive: DriveSubsystem, sew: SewSubsystem):
        exampleTrajectory = sew.readJson("blueidk2")
        init = exampleTrajectory.initialPose()
        drive.resetEncoders()
        drive.odometry.resetPosition(drive.sillyGyro.getRot(), 0, 0, init)

        self.ram = RamseteCommand(
            exampleTrajectory, drive.getPose2d, drive.ramsete, drive.feedforward,
            Ac.kDriveKinematics, drive.getCurrentSillySpeeds,
            PIDController(Ac.kPDriveVel, 0, 0),
            PIDController(Ac.kPDriveVel, 0, 0),
            drive.setVoltages, drive)
        
        super().__init__(self.ram)