/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    //left Motors
    private final CANSparkMax leftMaster = new CANSparkMax(
        DriveConstants.kLeftMasterPort, DriveConstants.kDriveMotorType);
    private final CANSparkMax leftSlave = new CANSparkMax(
        DriveConstants.kLeftSlavePort, DriveConstants.kDriveMotorType);
    private final CANSparkMax rightMaster = new CANSparkMax(
        DriveConstants.kRightMasterPort, DriveConstants.kDriveMotorType);
    private final CANSparkMax rightSlave = new CANSparkMax(
        DriveConstants.kRightSlavePort, DriveConstants.kDriveMotorType);


    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final Encoder leftDriveEncoder =
        new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1], 
        DriveConstants.kLeftEncoderReversed);

    private final Encoder rightDriveEncoder = 
        new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1], 
        DriveConstants.kRightEncoderReversed);
  
    private final DifferentialDrive defaultDrive = 
        new DifferentialDrive(leftMaster, rightMaster);

          // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  public DriveSubsystem() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(DriveConstants.kLeftMotorsInverted);
    leftSlave.setInverted(DriveConstants.kLeftMotorsInverted);
    rightMaster.setInverted(DriveConstants.kRightMotorsInverted);
    rightSlave.setInverted(DriveConstants.kRightMotorsInverted);

    leftMaster.setIdleMode(IdleMode.kBrake);
    leftSlave.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave.setIdleMode(IdleMode.kBrake);


    defaultDrive.setSafetyEnabled(false);

    leftDriveEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    rightDriveEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }

  public void setMotorsBrake() {
    leftMaster.setIdleMode(IdleMode.kBrake);
    leftSlave.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave.setIdleMode(IdleMode.kBrake);
  }

  public void setMotorsCoast() {
    leftMaster.setIdleMode(IdleMode.kCoast);
    leftSlave.setIdleMode(IdleMode.kCoast);
    rightMaster.setIdleMode(IdleMode.kCoast);
    rightSlave.setIdleMode(IdleMode.kCoast);
  }

  public double getDistAvg() {
    return (leftDriveEncoder.getDistance() + rightDriveEncoder.getDistance()) / 2;
  }

  public double getDistLeft() {
    return leftDriveEncoder.getDistance();
  }

  public double getDistRight() {
    return rightDriveEncoder.getDistance();
  }

  public double getAngle() {
    return gyro.getAngle();
  }

 /*
  public double distFromTarget() {
    return 90.75 / Math.tan(limelight.getTY() + 60.5);
  }
 */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftDriveEncoder.getRate(), rightDriveEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    defaultDrive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(-rightVolts);
    defaultDrive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    leftDriveEncoder.reset();
    rightDriveEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftDriveEncoder.getDistance() + rightDriveEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return leftDriveEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return rightDriveEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    defaultDrive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -gyro.getRate();
  }
}
