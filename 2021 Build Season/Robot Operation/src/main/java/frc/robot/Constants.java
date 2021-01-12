/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

        public final static int[] kLeftEncoderPorts = new int[] { 9, 8 };
        public final static int[] kRightEncoderPorts = new int[] { 7, 6 };
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

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
    }

    public final static class OIConstants {
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

    public final static class ShooterConstants {
        public static final int kShooterMotorID = 9;
        public static final MotorType kShooterMotorType = MotorType.kBrushless;
        public static final int kFeederMotorID = 3;
        public static final int kFeederBbRecPort = 0;
        public static final int kFeederBbBlastPort = 1;
        public static final int kShooterExitRecPort = 2;
        public static final int kShooterExitBlastPort = 3;
        public static final double kFeederMotorSpeed = 0.6;
        public static final double kFeederPreloadSpeed = 0.35;
        public static final double kP = 1;
        public static final int kI = 0;
        public static final int kD = 0;
        public static final int kFF = 0;
        public static final int kTargetRPM = 1500; // 3559 for 40 inches from LL, was set to 1500
        public static final int kMaxRPM = 5676;
        public static final double kToleranceRPM = 180;
        public static final int kBallsToExit = 3;
    }

    public final static class IndexerConstants {
        public final static int kIndexerLeftID = 0;
        public final static int kIndexerRightID = 1;
        public final static double kIndexerPower = 0.4;
    }

    public final static class IntakeConstants {
        public final static int kIntakeID = 2;
        public final static double kIntakeSpeed = -0.6;
    }

    public static class LimelightConstants {
        public final static double kTurnP = 0.01;
        public final static double kTurnI = 0;
        public final static double kTurnD = 0;
        public final static double kTargetAngle = 0;
        public final static double kDriveP = 0.01;
        public final static double kDriveI = 0;
        public final static double kDriveD = 0;
        public final static int kTurnToleranceDeg = 3;
        public final static int kDriveToleranceDeg = 3;
        public final static int kPracticePipeline = 0;
    }

    public static class ClimbConstants {
        public final static int kWinchID = 11;
    }
}
