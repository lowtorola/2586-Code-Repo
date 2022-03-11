// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * This class contains all drivebase constants & limelight constants, as the limelight is contained within
     * the drivetrain subsystem for lambda reasons!!
     */
    public static final class DriveConstants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.572;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.572;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(216.65); 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 21; // FIXME: Change this to fit into range
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(80.42); 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 8;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(137.81); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(272.55); // FIXME Measure and set back right steer offset
    }

    /**
     * This class contains all constants for the intake subsystem of the robot.
     */
    public static final class IntakeConstants {
        public static final int ROLLER_MOTOR = 1;
        public static final int[] CYLINDER = {8,7};
        public static final PneumaticsModuleType CYLINDER_MODULE_TYPE = PneumaticsModuleType.REVPH;
        public static final double FWD_SPEED = -0.85;
        public static final double REV_SPEED = 0.5;
    }

    public static final class ShooterConstants {
        public static final double MAX_VOLTS = 12.0;
        public static final int MAX_RPM = 5676;
        public static final double VOLT_PER_RPM = (double) MAX_VOLTS / MAX_RPM;
        public static final int FLYWHEEL = 13;
        public static final int FEEDER = 0;
        public static final int TOP_BB = 0;
        public static final int BOTTOM_BB = 1;
        public static final int SHOOT_RPM = 1850; // change this to change voltage output
        public static final double SHOOT_VOLTS = VOLT_PER_RPM * SHOOT_RPM + 0.25;
        public static final double FEEDER_FWD = 0.8;
        public static final double FEEDER_INDEX = 0.5;
        public static final double FEEDER_REV = -0.4;
    }

    public static final class ClimbConstants {

        public static final int LEFT_TELESCOPE = 14;
        public static final int RIGHT_TELESCOPE = 15;
        public static final double WINCH_SPEED = 0.8;

        public static final double MIN_HEIGHT = 0; // rotations
        public static final double MAX_HEIGHT = 80; // rotations
        public static final int STAGE_HEIGHT = 14; // rotations: find actual stage height

        public static final int[] PIVOT = {6,9};

        public static final int SMART_MOTION_SLOT = 0;
        public static final double KP_LEFT = 0.0004; // FIXME: plug in tuned values for all these!!
        public static final double KP_RIGHT = 0.00045; // FIXME: plug in tuned values for all these!!
        public static final int KI = 0;
        public static final double KD = 0.0; // 0.001
        public static final double KIZ = 0;
        public static final double KFF_LEFT = 0.00046; 
        public static final double KFF_RIGHT = 0.00046;
        public static final double KMAX_OUTPUT = 0.9;
        public static final double KMIN_OUTPUT = -1.0;
        public static final int MAX_RPM = 5700;
        public static final int MAX_VEL = 5000;
        public static final int MIN_VEL = 0;
        public static final int MAX_ACC = 2000;
        public static final double ALLOWED_ERR_LEFT = 0.5; // 1 rotation for real climb
        public static final double ALLOWED_ERR_RIGHT = 0.1;
    }

    /**
     * This class contains all operator interface (OI) constants for the driver DS4, operator DS4, and fight
     * stick. Port numbers are from Driver Station, DS4 and button board constants are subclassed.
     */
    public static final class OIConstants {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
        public static final int FIGHT_STICK = 2;

        /**
         * This class contains button mapping for the Dualshock 4 controllers.
         */
        public static final class DS4 {
            // Buttons / axis clicks
            public static final int SQUARE = 1;
            public static final int X = 2;
            public static final int CIRCLE = 3;
            public static final int TRIANGLE = 4;
            public static final int R_BUMPER = 5;
            public static final int L_BUMPER = 6;
            public static final int R_TRIGBUTTON = 7;
            public static final int L_TRIGBUTTON = 8;
            public static final int SHARE = 9;
            public static final int OPTIONS = 10;
            public static final int L_STICK = 11;
            public static final int R_STICK = 12;
            public static final int PS_LOGO = 13;
            public static final int CENTER_PAD = 14;
            // Axes
            public static final int L_STICK_X = 0; // left is -1
            public static final int L_STICK_Y = 1; // forward is -1
            public static final int R_STICK_X = 2; // left is -1
            public static final int R_STICK_Y = 5; // forward is -1
            public static final int L_TRIGGER = 3; // -1 at rest, scales to 1
            public static final int R_TRIGGER = 4; // -1 at rest, scales to 1
        }
        /**
         * This class contains button bindings for the Fight Stick
         */
        public static class FightStick {
            // Buttons
            public static final int A = 1;
            public static final int B = 2;
            public static final int X = 3;
            public static final int Y = 4;
            public static final int L_BUMPER = 5;
            public static final int R_BUMPER = 6;
            public static final int SHARE = 7;
            public static final int OPTIONS = 8;
            public static final int L3 = 9; // DO NOT USE!!!
            public static final int R3 = 10; // DO NOT USE!!!
            // Axes
            public static final int X_AXIS = 0; // Left = -1, Center = 0, Right = 1
            public static final int Y_AXIS = 1; // Forward = -1, Center = 0, Back = 1
            public static final int L_TRIGGER = 2; // Rest = 0, Pressed = 1
            public static final int R_TRIGGER = 3; // Rest = 0, Pressed = 1

        }

    }

}
