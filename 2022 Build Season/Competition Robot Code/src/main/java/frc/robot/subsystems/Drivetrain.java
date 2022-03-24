// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    FRONT_LEFT_MODULE_STEER_ENCODER, false, 2.993, 0.632, 2.200, 0.304, -214.52, 4.549, 0.03, 0.805, 0.228, 0.006); //kD is .18

  private final SwerveModule m_frontRight = new SwerveModule(FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR, 
    FRONT_RIGHT_MODULE_STEER_ENCODER, false, 2.993, 0.632, 2.200, 0.304, -79.78, 4.832, 0.03, 0.870, 0.236, 0.007); //kD is .22

  private final SwerveModule m_backLeft = new SwerveModule(BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR, 
    BACK_LEFT_MODULE_STEER_ENCODER, false, 2.993, 0.632, 2.200, 0.304, -138.51, 4.784, 0.03, 0.911, 0.236, 0.007); //kD is .22

  private final SwerveModule m_backRight = new SwerveModule(BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR, 
    BACK_RIGHT_MODULE_STEER_ENCODER, false, 2.993, 0.632, 2.200, 0.304, -270.45, 4.900, 0.03, 0.884, 0.232, 0.007); //kD is .23

  // private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  public static final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0), 
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

  public final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, robotRotation2d());

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
    if(fieldRelative) {
    m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d());
    } else {
      m_chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    }
  }

  public void driveFromSpeeds(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
    if(fieldRelative) {
      m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, robotRotation2d());
    } else {
    m_chassisSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        robotRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }
  /** Sets the robot odometry to a given Pose2d */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, robotRotation2d());
  }

    /**
   * Returns the currently-estimated pose of the robot.
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getGyroAngle() {
    return -1.0 * m_gyro.getAngle();
  }
  public Rotation2d robotRotation2d() {
    return new Rotation2d(Math.toRadians(getGyroAngle()));
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

    // Update the odometry in the periodic block
    updateOdometry();

    SmartDashboard.putNumber("Front Left Angle", Math.toDegrees(m_frontLeft.getTurnAngle()));
    SmartDashboard.putNumber("Front Right Angle", Math.toDegrees(m_frontRight.getTurnAngle()));
    SmartDashboard.putNumber("Back Left Angle", Math.toDegrees(m_backLeft.getTurnAngle()));
    SmartDashboard.putNumber("Back Right Angle", Math.toDegrees(m_backRight.getTurnAngle()));
    SmartDashboard.putNumber("Target Front Left Angle", Math.toDegrees(m_frontLeft.moduleState().angle.getRadians()));
    SmartDashboard.putNumber("Target Front Right Angle", Math.toDegrees(m_frontRight.moduleState().angle.getRadians()));
    SmartDashboard.putNumber("Target Back Left Angle", Math.toDegrees(m_backLeft.moduleState().angle.getRadians()));
    SmartDashboard.putNumber("Target Back Right Angle", Math.toDegrees(m_backLeft.moduleState().angle.getRadians()));
    SmartDashboard.putNumber("Gyro Return", getGyroAngle());

  }

}