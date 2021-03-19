/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public final static int kLeftMasterPort = 1;
        public final static int kLeftSlavePort = 2;
        public final static int kRightMasterPort = 3;
        public final static int kRightSlavePort = 4;

        public final static double kDriveDeadband = 0.05;

        public final static boolean kLeftMotorsInverted = false;
        public final static boolean kRightMotorsInverted = false;

        public final static MotorType kDriveMotorType = MotorType.kBrushless;

        public final static int[] kLeftEncoderPorts = new int[] { 5, 4 };
        public final static int[] kRightEncoderPorts = new int[] { 7, 6 };
        public static final boolean kLeftEncoderReversed = true;
        public static final boolean kRightEncoderReversed = false;

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR * 0.48;

        public static final double kDriveP = 0;
        public static final double kDriveI = 0;
        public static final double kDriveD = 0;
        public static final double kTurnP = 0;
        public static final double kTurnI = 0;
        public static final double kTurnD = 0;

        // Trajectory constants

    // characterization constants
    public static final double ksVolts = 0.221;
    public static final double kvVoltSecondsPerMeter = 2.3;
    public static final double kaVoltSecondsSquaredPerMeter = 0.47;
    public static final double kPDriveVel = 1.7; // default to 2.59

    // DifferentialDriveKinematics values
    public static final double kTrackwidthMeters = 1.61; // might need to be 1.61 because of off-center turn
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    // Max Trajectory Velocity/Acceleration
    public static final double kMaxSpeedMetersPerSecond = 4; // robot max speed is 5.22 m/s
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;

    // Ramsete parameters
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2.0;
    public static final double kRamseteZeta = 0.9;
    }

    public final static class OIConstants {

        public static final int kSquareButton = 1;
        public static final int kXButton = 2;
        public static final int kCircleButton = 3;
        public static final int kTriangleButton = 4;
        public static final int kLeftBumper = 5;
        public static final int kRightBumper = 6;
        public static final int kLeftTrigger = 7;
        public static final int kRightTrigger = 8;
        public static final int kShareButton = 9;
        public static final int kOptionsButton = 10;
        public static final int kLeftStick = 11;
        public static final int kRightStick = 12;
        public static final int kPSLogo = 13;
        public static final int kCenterButton = 14;

        public static final int kDriveControllerPort = 0; 
        public static final int kShooterOnButton = 3; // O
        public static final int kFeederOnButton = 5; // Left Bumper
        public static final int kFeederPreloadButton = 14; // Center pad
        public static final int kIntakeOnButton = 7; // Left Trigger
        public static final int kIntakeDeployButton = 9; // Share
        public static final int kIntakeRetractButton = 10; // Options
        public static final int kLimelightAimButton = 6; // Right Bumper
        public static final int kClimbPowerButton = 4; // Triangle
        public static final int kShooterAutoButton = 2; // X
    }

    public final static class LimelightConstants {
        public static final int kDisabledPipeline = 0; // Everything off, increase exposure
        public static final int kCompTarget = 1; // Competition target at comp height
        public static final int kPracticeTarget = 2; // Two vertical lines at 71 inches high 

        public static final double kP = 0.075;
        public static final double kI = 0.0;
        public static final double kD = 0.008; 
        public static final double kTurnToleranceDeg = 0.3;
        public static final double kTargetAngle = 0.0;

        public static final double kLLheight = 19;
        public static final double kTargetHeight = 71; // change if target changes
        public static final double kCalcHeight = kTargetHeight - kLLheight;
        public static final double kBaseDegree = 20.0; // TODO: this needs to be changed to the mounting angle (possibly 60 degrees)
        public static final double kLimelightOffset = 18.0;

        public static final double kInputSquared = .168184;
        public static final double kInputScaled = -7.7377;
        public static final double kConstant = 154.503;
    }

    public final static class ShooterConstants {
        
        public static final int kShooterMotorID = 11;
        public static final int kBeltMotorID = 5;
        public static final int[] kHoodPistonID = {1, 3};
        public static final MotorType kShooterMotorType = MotorType.kBrushless;
        public static final int kFeederMotorID = 3;

        public static final int kFeederBbRecPort = 0;
        public static final int kFeederBbBlastPort = 1;
        public static final int kShooterExitRecPort = 2;
        public static final int kShooterExitBlastPort = 3;

        public static final double kFeederMotorSpeed = 0.45;
        public static final double kFeederPreloadSpeed = 0.35;
        public static final double kBeltPreloadSpeed = 0.6; // TODO: test to determine optimal speed
        public static final double kBeltMotorSpeed = 0.7;

        public static final double kP = .0002; // should be 1.63
        public static final int kI = 0;
        public static final double kD = 0.0;

        public static final double kvVoltSecondsPerRotation = 0.132; // should be .132
        public static final double ksVolts = 0.288; // should be .174
        public static final double kaVoltSecondsSquaredPerRotation = 0.0297; // should be .0387

        public static final int kTargetRPM = 4300;

        public static final int kMaxRPM = 5676; // was 750
        public static final double kToleranceRPM = 100;

        public static final int kBallsToExit = 3;

        public static final double kQuadraticNear = 4.05;
		public static final double kLinearNear = -10.005;
        public static final double kConstantNear = 4100;
        
        public static final double kQuadraticFar = 2.9975;
        public static final double kLinearFar = -11.3067;
        public static final double kConstantFar = 4603.4246;

		public static final double kAngleErrorSwitcher = 10;
    }

    public final static class IndexerConstants {
        public final static int kIndexerLeftID = 0;
        public final static int kIndexerRightID = 1;
        public final static double kIndexerPower = 0.55;
    }

    public final static class IntakeConstants {
        public final static int kIntakeID = 2;
        public final static double kIntakeSpeed = -0.6;
        public static final int[] kIntakePistonID = {2, 0};
    }

    public static class ClimbConstants {
        public final static int kWinchID = 11;
    }
}
