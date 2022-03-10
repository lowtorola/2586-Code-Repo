// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveModule;
import static frc.robot.Constants.DriveConstants.*;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
  SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveModule m_frontLeft = new SwerveModule(FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR,
    FRONT_LEFT_MODULE_STEER_ENCODER, false, 0.672, 0.702, 2.69, -214.52, 3.0, 0.43, 0.24);

  private final SwerveModule m_frontRight = new SwerveModule(FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR, 
    FRONT_RIGHT_MODULE_STEER_ENCODER, false, 0.672, 0.702, 2.69, -79.78, 3.0, 0.43, 0.24);

  private final SwerveModule m_backLeft = new SwerveModule(BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR, 
    BACK_LEFT_MODULE_STEER_ENCODER, false, 0.672, 0.702, 2.69, -138.51, 3.0, 0.43, 0.24);

  private final SwerveModule m_backRight = new SwerveModule(BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR, 
    BACK_RIGHT_MODULE_STEER_ENCODER, false, 0.672, 0.702, 2.69, -270.45, 3.0, 0.43, 0.24);

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  public static final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0), 
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

  public final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

  // Class-wide desired chassis speeds
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // var swerveModuleStates =
    //     m_kinematics.toSwerveModuleStates(
    //         fieldRelative
    //             ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
    //             : new ChassisSpeeds(xSpeed, ySpeed, rot));
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_VELOCITY_METERS_PER_SECOND);
    // m_frontLeft.setDesiredState(swerveModuleStates[0]);
    // m_frontRight.setDesiredState(swerveModuleStates[1]);
    // m_backLeft.setDesiredState(swerveModuleStates[2]);
    // m_backRight.setDesiredState(swerveModuleStates[3]);
    if(fieldRelative) {
    m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d());
    } else {
      m_chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    }
  }

  public void driveFromSpeeds(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }

  public SwerveModule getFrontLeft() {
    return m_frontLeft;
  }

  public SwerveModule getFrontRight() {
    return m_frontRight;
  }

  public SwerveModule getBackLeft() {
    return m_backLeft;
  }

  public SwerveModule getBackRight() {
    return m_backRight;
  }

  public double getGyroAngle() {
    return m_gyro.getAngle();
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);

    SmartDashboard.putNumber("Front Left Angle", Math.toDegrees(m_frontLeft.getTurnAngle()));
    SmartDashboard.putNumber("Front Right Angle", Math.toDegrees(m_frontRight.getTurnAngle()));
    SmartDashboard.putNumber("Back Left Angle", Math.toDegrees(m_backLeft.getTurnAngle()));
    SmartDashboard.putNumber("Back Right Angle", Math.toDegrees(m_backRight.getTurnAngle()));
    SmartDashboard.putNumber("Target Front Left Angle", Math.toDegrees(m_frontLeft.moduleState().angle.getRadians()));
    SmartDashboard.putNumber("Target Front Right Angle", Math.toDegrees(m_frontRight.moduleState().angle.getRadians()));
    SmartDashboard.putNumber("Target Back Left Angle", Math.toDegrees(m_backLeft.moduleState().angle.getRadians()));
    SmartDashboard.putNumber("Target Back Right Angle", Math.toDegrees(m_backLeft.moduleState().angle.getRadians()));

  }

}