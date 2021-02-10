// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    // Trajectory constants

    // characterization constants
    public static final double ksVolts = 0.221;
    public static final double kvVoltSecondsPerMeter = 2.3;
    public static final double kaVoltSecondsSquaredPerMeter = 0.47;
    public static final double kPDriveVel = 2.59;

    // DifferentialDriveKinematics values
    public static final double kTrackwidthMeters = 1.61; // might need to be 1.61 because of off-center turn
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    // Max Trajectory Velocity/Acceleration
    public static final double kMaxSpeedMetersPerSecond = 4; // robot max speed is 5.22 m/s
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;

    // Ramsete parameters
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

		public static final int kLeftMotor1Port = 1;
        public static final int kLeftMotor2Port = 2;
        
		public static final int kRightMotor1Port = 3;
        public static final int kRightMotor2Port = 4;
        
		public static final int[] kLeftEncoderPorts = {5, 4};
		public static final boolean kLeftEncoderReversed = true;
		public static final int[] kRightEncoderPorts = {7, 6};
        public static final boolean kRightEncoderReversed = false;
        
        public static final double kWheelDiameterMeters = 0.1524; // 6 inch wheel
        public static final double kEncoderCPR = 1024;
        public static final double kGearing = 0.48; 

		public static final double kEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / kEncoderCPR * kGearing;

    }

}
