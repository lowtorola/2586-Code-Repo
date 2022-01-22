// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
/*
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
*/
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import frc.robot.SwerveModule;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = 1.5 * Math.PI; // 1/4 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.279, 0.279);
  private final Translation2d m_frontRightLocation = new Translation2d(0.279, -0.279);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.279, 0.279);
  private final Translation2d m_backRightLocation = new Translation2d(-0.279, -0.279);

  private final SwerveModule m_frontLeft = new SwerveModule(2, 1, 9, false, 0.672,0.702,2.69,0.17,1.00,0.62,0.24);
  private final SwerveModule m_frontRight = new SwerveModule(6, 5, 11, true, 0.672,0.702,2.69,-0.07,0.98,0.62,0.24);
  private final SwerveModule m_backLeft = new SwerveModule(4, 3, 10, false, 0.672,0.702,2.69,0.98,0.98,0.62,0.24);
  private final SwerveModule m_backRight = new SwerveModule(8, 7, 12, true, 0.672,0.702,2.69,-0.85,1.02,0.62,0.24);

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

  public Drivetrain() {
    m_gyro.reset();
    m_frontLeft.zeroEncoders();
    m_frontRight.zeroEncoders();
    m_backLeft.zeroEncoders();
    m_backRight.zeroEncoders();
    m_frontRight.invertDrive(false);
    m_frontLeft.invertDrive(true);
    m_backRight.invertDrive(true);
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
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
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

}
