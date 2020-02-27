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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.lib.limelight;

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

    private final limelight limelight = new limelight();

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final Encoder leftDriveEncoder =
        new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1], 
        DriveConstants.kLeftEncoderReversed);

    private final Encoder rightDriveEncoder = 
        new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1], 
        DriveConstants.kRightEncoderReversed);
  
    private final DifferentialDrive defaultDrive = 
        new DifferentialDrive(leftMaster, rightMaster);

    private double deadBand = 0;

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

    limelight.setLEDMode(0);
    limelight.setCAMMode(0);
    limelight.setPipeline(0);

    leftDriveEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    rightDriveEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
  }

  public void arcadeDrive(double fwd, double rot) {
      defaultDrive.arcadeDrive(fwd, rot);
      defaultDrive.setDeadband(deadBand);
  }

  public void resetEncoders() {
      leftDriveEncoder.reset();
      rightDriveEncoder.reset();
  }

  public void setDeadband(double dB) {
    deadBand = dB;
  }

  public double getLimelightAngleDegX() {
    return limelight.getTX();
  }

  public double getLimelightAngleDegY() {
    return limelight.getTY();
  }

  public double getDistAvg() {
    return (leftDriveEncoder.getDistance() + rightDriveEncoder.getDistance()) / 2;
  }

  public double getAngle() {
    return gyro.getAngle();
  }

  public void limelightAimConfig(double pipeline) {
    limelight.setLEDMode(0);
    limelight.setCAMMode(0);
    limelight.setPipeline(pipeline);
  }

  public void limelightDriveConfig() {
    limelight.setLEDMode(1);
    limelight.setCAMMode(1);
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
}
