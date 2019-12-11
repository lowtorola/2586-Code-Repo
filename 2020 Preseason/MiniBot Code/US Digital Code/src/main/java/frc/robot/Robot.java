/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  Encoder leftUSD;
  Encoder rightUSD;

  CANEncoder leftNEO;
  CANEncoder rightNEO;

  Joystick joy1;

  CANSparkMax leftMaster;
  CANSparkMax leftSlave;
  CANSparkMax rightMaster;
  CANSparkMax rightSlave;

  DifferentialDrive drive;

  final double kInchesPerRevolution = 12.56;

  final double gear_Ratio = 4.16;

  @Override
  public void robotInit() {

    final double usd_CountsPerRevolution = 1440;
    final double usd_InchesPerCount = kInchesPerRevolution / usd_CountsPerRevolution;

    leftUSD = new Encoder(7, 6);
    leftUSD.setDistancePerPulse(.017);
    leftUSD.reset();

    rightUSD = new Encoder(9, 8);
    rightUSD.setDistancePerPulse(.017);
    rightUSD.setReverseDirection(true);
    rightUSD.reset();

    joy1 = new Joystick(0);

    leftMaster = new CANSparkMax(3, MotorType.kBrushless);
    leftSlave = new CANSparkMax(2, MotorType.kBrushless);
    rightMaster = new CANSparkMax(6, MotorType.kBrushless);
    rightSlave = new CANSparkMax(5, MotorType.kBrushless);

    leftMaster.setIdleMode(IdleMode.kBrake);
    leftMaster.setInverted(false);
    leftSlave.setIdleMode(IdleMode.kBrake);
    leftSlave.setInverted(false);
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave.setIdleMode(IdleMode.kBrake);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftNEO = leftMaster.getEncoder();
    leftNEO.setPosition(0);
    rightNEO = rightMaster.getEncoder();
    rightNEO.setPosition(0);

    drive = new DifferentialDrive(leftMaster, rightMaster);
    drive.setDeadband(0.2);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    sensorPrints();

    if (joy1.getRawButton(1)) {
      leftUSD.reset();
      rightUSD.reset();
      leftNEO.setPosition(0);
      rightNEO.setPosition(0);
    }

    double drivePower = -joy1.getRawAxis(1);
    double driveSteer = -joy1.getRawAxis(2);
    drive.arcadeDrive(drivePower, driveSteer, true);

  }

  @Override
  public void testInit() {
  }

  public void sensorPrints() {

    SmartDashboard.putNumber("Left USD Distance", leftUSD.getDistance());
    SmartDashboard.putNumber("Right USD Distance", rightUSD.getDistance());

    SmartDashboard.putNumber("Left USD Ticks", leftUSD.getRaw());
    SmartDashboard.putNumber("Right USD Ticks", rightUSD.getRaw());

    SmartDashboard.putNumber("Left USD Sampling", leftUSD.getEncodingScale());
    SmartDashboard.putNumber("Right USD Sampling", rightUSD.getEncodingScale());

    SmartDashboard.putNumber("Left NEO Distance", neoDistConv(-leftNEO.getPosition()));
    SmartDashboard.putNumber("Right NEO Distance", neoDistConv(rightNEO.getPosition()));

    SmartDashboard.putNumber("Left NEO Position", -leftNEO.getPosition());
    SmartDashboard.putNumber("Right NEO Position", rightNEO.getPosition());

  }

  public double neoDistConv (double x) {
    x /= gear_Ratio;
    x *= kInchesPerRevolution;
    return x;
  }

  @Override
  public void testPeriodic() {
  }

}