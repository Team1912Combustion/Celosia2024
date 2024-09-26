#
# Copyright (c) FIRSTfrom pathplannerlib.auto import AutoBuilder
import pathplannerlib
from pathplannerlib.config import *
from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.config import *
from pathplannerlib.auto import AutoBuilder
from wpilib import DriverStation
import wpilib.drive
import wpimath.estimator
import commands2
import math
import navx
import wpimath
import wpimath.geometry
from  wpilib.shuffleboard import *
import constants
import wpimath.kinematics
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
import wpimath.units

from wpimath.controller import RamseteController, SimpleMotorFeedforwardMeters
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, Trajectory
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint

from custom.ramsetecommand import RamseteCommand

from wpimath.controller import PIDController

from wpilib.sysid import SysIdRoutineLog
from commands2.sysid import SysIdRoutine
from commands2 import Command

from wpilib import RobotController, Encoder, MotorControllerGroup
from wpilib.drive import DifferentialDrive
from wpimath.geometry import Pose2d
from wpimath.estimator import DifferentialDrivePoseEstimator

from subsystems.opticalsubsystem import OpticalSubsystem
from subsystems.limesubsystem import LimeSubsystem

from subsystems.gyrosub import *

from constants import DriveConstants as Dc, AutoConstants as Ac, OpticalConstants as Oc, LimeConstants as Lc
from wpilib import RobotController

from phoenix5 import WPI_VictorSPX

from commands2 import cmd, RunCommand

from wpilib.shuffleboard import Shuffleboard

from wpilib import SmartDashboard

# 9982 - the intake falls to the front of the robot

class DriveSubsystem(commands2.Subsystem):
    def __init__(self, opticalSubsystem: OpticalSubsystem, limeSubsystem: LimeSubsystem, gyro: GyroSub) -> None:
        super().__init__()

        self._eyes = opticalSubsystem
        self._lime = limeSubsystem
        self.sillyGyro = gyro

        self.isRewindTime = False

        self.leftFrontMotor = WPI_VictorSPX(Dc.kLeftFrontMotorPort)
        self.leftRearMotor = WPI_VictorSPX(Dc.kLeftRearMotorPort)

        self.rightFrontMotor = WPI_VictorSPX(Dc.kRightFrontMotorPort)
        self.rightRearMotor = WPI_VictorSPX(Dc.kRightRearMotorPort)

        self.rightRearMotor.follow(self.rightFrontMotor)
        self.leftRearMotor.follow(self.leftFrontMotor)

        self.leftMotorGroup = MotorControllerGroup(self.leftFrontMotor)

        self.rightMotorGroup = MotorControllerGroup(self.rightFrontMotor)
        self.rightMotorGroup.setInverted(True)

        self.drive = DifferentialDrive(self.leftMotorGroup, self.rightMotorGroup)

        self.leftEncoder = Encoder(
            Dc.kLeftEncoderPorts[0],
            Dc.kLeftEncoderPorts[1],
            Dc.kLeftEncoderReversed)

        self.rightEncoder = Encoder(
            Dc.kRightEncoderPorts[0],
            Dc.kRightEncoderPorts[1],
            Dc.kRightEncoderReversed)

        self.leftEncoder.setDistancePerPulse(Dc.kEncoderDistancePerPulse)
        self.rightEncoder.setDistancePerPulse(Dc.kEncoderDistancePerPulse)
        
        self.leftEncoder.reset()
        self.rightEncoder.reset()

        iRot = self.sillyGyro.getRot()
        iPose = Pose2d(0, 0, iRot)

        self.odometry = DifferentialDrivePoseEstimator(Ac.kDriveKinematics, iRot, 0, 0, iPose)

        self.ramsete = RamseteController(Ac.kRamseteB, Ac.kRamseteZeta)

        self.feedforward = SimpleMotorFeedforwardMeters(
                kS=Ac.ksVolts,
                kV=Ac.kvVoltSecondsPerMeter,
                kA=Ac.kaVoltSecondsSquaredPerMeter)

        self.voltConstraint = DifferentialDriveVoltageConstraint(
            self.feedforward,
            Ac.kDriveKinematics,
            maxVoltage=10)

        self.trajectoryConfig = TrajectoryConfig(
            Ac.kMaxSpeedMetersPerSecond,
            Ac.kMaxAccelerationMetersPerSecondSquared)

        self.trajectoryConfig.setKinematics(Ac.kDriveKinematics)
        self.trajectoryConfig.addConstraint(self.voltConstraint)

        self.idk = Shuffleboard.getTab("drive")
        self.idk2 = self.idk.add("encoderleft", 0)
        self.idk3 = self.idk.add("encoderright", 0)
        self.idk5 = self.idk.add("odometryX",0)
        self.idk6 = self.idk.add("odometryY", 0)
        self.idk7 = self.idk.add("odometryDeg", 0)
        self.idk8 = self.idk.add("limeTag", 0)
        self.idk9 = self.idk.add("limeLatency", 0)
        self.idk10 = self.idk.add("rawlimepose", [0, 0, 0, 0])

    def periodic(self):
        self.odometry.update(self.sillyGyro.getRot(), self.leftEncoder.getDistance(), self.rightEncoder.getDistance())

        if self._eyes.bluPose != None:
            pass
            #self.odometry.addVisionMeasurement(self._eyes.bluPose.estimatedPose.toPose2d(), self._eyes.bluPose.timestampSeconds, Oc.SKEPTICISM)

        if self._eyes.grnPose != None:
            pass
            #self.odometry.addVisionMeasurement(self._eyes.grnPose.estimatedPose.toPose2d(), self._eyes.grnPose.timestampSeconds, Oc.SKEPTICISM)

        if self._lime.botPose != None:
            pass
            #self.idk10.getEntry().setFloatArray(self._lime.botPose)
            #fpgatime = RobotController.getFPGATime()
            #self.odometry.addVisionMeasurement(self._lime.seqToPose(self._lime.botPose), self._lime.calcTimestamp(fpgatime), Lc.SKEPTICISM)

        #print(self._lime.totalLatency)
        self.idk2.getEntry().setInteger(self.leftEncoder.get())
        self.idk3.getEntry().setInteger(self.rightEncoder.get())
        self.idk5.getEntry().setFloat(self.odometry.getEstimatedPosition().X())
        self.idk6.getEntry().setFloat(self.odometry.getEstimatedPosition().Y())
        self.idk7.getEntry().setFloat(self.odometry.getEstimatedPosition().rotation().degrees())
        #self.idk9.getEntry().setFloat(self._lime.totalLatency)
       # self.idk8.getEntry().setInteger(self._lime.priTag)
        #self.idk10.getEntry().setFloatArray(self._lime.botPose)
        #self.idk9.getEntry().setString("Hehe")

    def arcadeDrive(self, fwd: float, rot: float):
        if self.isRewindTime:
            fwd *= -1

        """
        Drives the robot using arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        """

        self.drive.arcadeDrive(fwd, rot, squareInputs=False)

    def arcade(self, fwd: float, rot: float):
        if self.isRewindTime: fwd *= -1

        self.drive.arcadeDrive(fwd, rot, squareInputs=True)

    def resetEncoders(self):
        """Resets the drive encoders to currently read a position of 0."""
        self.leftEncoder.reset()
        self.rightEncoder.reset()

    def getAverageEncoderDistance(self):
        """
        Gets the average distance of the two encoders.

        :returns: the average of the two encoder readings
        """
        return (self.leftEncoder.getDistance() + self.rightEncoder.getDistance()) / 2.0

    def getLeftEncoder(self) -> wpilib.Encoder:
        """
        Gets the left drive encoder.

        :returns: the left drive encoder
        """
        return self.leftEncoder

    def getRightEncoder(self) -> wpilib.Encoder:
        """
        Gets the right drive encoder.

        :returns: the right drive encoder
        """
        return self.rightEncoder

    def setMaxOutput(self, maxOutput: float):
        """
        Sets the max output of the drive. Useful for scaling the drive to drive more slowly.

        :param maxOutput: the maximum output to which the drive will be constrained
        """
        self.drive.setMaxOutput(maxOutput)

    def zeroHeading(self):
        """
        Zeroes the heading of the robot.
        """
        self.sillyGyro.reset()


    def getPose2d(self) -> wpimath.geometry.Pose2d:
        """
        Returns the current Pose2d of the robot.  Based on PoseEstimator
        returns: postEstimator Pose2d
        """
        #pose = wpimath.geometry.Pose2d(self.getGyroRotation2d(),0,0)
        #pose = wpimath.geometry.Pose2d(self.odometry.getEstimatedPosition())
        pose = self.odometry.getEstimatedPosition()
        return pose


    def getHeading(self) -> float:
        """
        Returns the heading of the robot.
        :returns: the robot's heading in degrees, from 180 to 180
        """
        #return math.remainder(self.gyro.getAngle(), 180) * (
        #    -1 if constants.DriveConstants.kGyroReversed else 1
        #)

        # Convert Radian-based heading to degrees
        return self.sillyGyro.getRot(True)
        

    def getTurnRate(self):
        """
        Returns the turn rate of the robot.

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.sillyGyro.getRate() * (
            -1 if constants.DriveConstants.kGyroReversed else 1
        )

    def getLeftVelocity(self):
        return self.leftEncoder.getVelocity()
    
    def getRightVelocity(self):
        return self.leftEncoder.getVelocity()


    def getCurrentSpeeds(self):
        speeds = wpimath.kinematics.DifferentialDriveWheelSpeeds()
        return constants.AutoConstants.kDriveKinematics.toChassisSpeeds(speeds)
    
    def getCurrentSillySpeeds(self):
        return wpimath.kinematics.DifferentialDriveWheelSpeeds()

    # === sysid ===
    def setVoltagesBoth(self, both):
        self.leftMotorGroup.setVoltage(both)
        self.rightMotorGroup.setVoltage(both)

    def logMotors(self, routinelog: SysIdRoutineLog):
        routinelog.motor("drive-left") \
            .voltage(self.leftMotorGroup.get() * RobotController.getBatteryVoltage()) \
            .position(self.leftEncoder.getDistance()) \
            .velocity(self.leftEncoder.getRate())
        
        routinelog.motor("drive-right") \
            .voltage(self.rightMotorGroup.get() * RobotController.getBatteryVoltage()) \
            .position(self.rightEncoder.getDistance()) \
            .velocity(self.rightEncoder.getRate())
    # === /sysid ===

    def setVoltages(self, left, right):
        self.leftMotorGroup.setVoltage(left)
        self.rightMotorGroup.setVoltage(right)

    def stopIt(self):
        return cmd.runOnce(lambda: self.setVoltages(0, 0), self)

    def shouldFlipPath() -> bool:
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
    
    def silly(self, trajectory: Trajectory):
        return RamseteCommand(
            trajectory, self.getPose2d, self.ramsete, self.feedforward,
            Ac.kDriveKinematics, self.getCurrentSpeeds,
            PIDController(Ac.kPDriveVel, 0, 0),
            PIDController(Ac.kPDriveVel, 0, 0),
            10, self)

    def toggleRewindTime(self):
        return cmd.runOnce(self._toggleRewindTime, self)

    def _toggleRewindTime(self):
        self.isRewindTime = not self.isRewindTime