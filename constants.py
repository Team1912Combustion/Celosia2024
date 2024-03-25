
import math
from wpimath.kinematics import DifferentialDriveKinematics

class DriveConstants:
    kLeftMotorPort = 1
    kRightMotorPort = 0

    kLeftEncoderPorts = (2, 3)
    kRightEncoderPorts = (0, 1)
    kLeftEncoderReversed = False
    kRightEncoderReversed = True

    # Encoder counts per revolution/rotation.
    kEncoderCPR = 360
    kWheelDiameterMeters = 0.152

    # Assumes the encoders are directly mounted on the wheel shafts
    kEncoderDistancePerPulse = (kWheelDiameterMeters * math.pi) / kEncoderCPR

    kGyroReversed = False

    kStabilizationP = 1
    kStabilizationI = 0.5
    kStabilizationD = 0

    kTurnP = .01
    kTurnI = 0
    kTurnD = 0

    kMaxTurnRateDegPerS = 100
    kMaxTurnAccelerationDegPerSSquared = 300

    kTurnToleranceDeg = 5
    kTurnRateToleranceDegPerS = 10  # degrees per second

class AutoConstants:

    # In meters, distance between wheels on each side of robot.
    kTrackWidthMeters = 0.60
    kDriveKinematics = DifferentialDriveKinematics(kTrackWidthMeters)

    # Encoder counts per revolution/rotation.
    kEncoderCPR = 360
    kWheelDiameterMeters = 0.152

    # NOTE: Please do NOT use these values on your robot. Rather, characterize your
    # drivetrain using the FRC Characterization tool. These are for demo purposes
    # only!
    ksVolts = 2.155
    kvVoltSecondsPerMeter = 2.5377
    kaVoltSecondsSquaredPerMeter = 1.1553

    # The P gain for our turn controllers.
    kPDriveVel = 1

    # The max velocity and acceleration for our autonomous.
    kMaxSpeedMetersPerSecond = .5
    kMaxAccelerationMetersPerSecondSquared = .5

    # Baseline values for a RAMSETE follower in units of meters
    # and seconds. These are recommended, but may be changes if wished.
    kRamseteB = 2
    kRamseteZeta = 0.7

    # The number of motors on the robot.
    kDrivetrainMotorCount = 4

class OIConstants:
    kDriverControllerPort = 0
    kDriverRightBumper = 6
    kDriverLeftBumper = 5
    kDriverXbutton = 3
    kDriverYbutton =  4