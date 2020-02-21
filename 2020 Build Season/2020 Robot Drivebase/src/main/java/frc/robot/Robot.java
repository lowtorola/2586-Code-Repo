/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
 
  CANSparkMax leftMaster, leftSlave, rightMaster, rightSlave, shooter;
  CANEncoder shooterEncoder;
  Spark shooterFeeder;

  Joystick stick;

  DifferentialDrive drive;
  
  @Override
  public void robotInit() {

    leftMaster = new CANSparkMax(1, MotorType.kBrushless);
    leftSlave = new CANSparkMax(2, MotorType.kBrushless);
    rightMaster = new CANSparkMax(3, MotorType.kBrushless);
    rightSlave = new CANSparkMax(4, MotorType.kBrushless);
    shooter = new CANSparkMax(9, MotorType.kBrushless);

    shooterEncoder = shooter.getEncoder();

    shooterFeeder = new Spark(3);

    stick = new Joystick(0);

    // leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    leftMaster.setInverted(false);
    leftSlave.setInverted(false);
    rightMaster.setInverted(false);

    drive = new DifferentialDrive(leftMaster, rightMaster);
    drive.setSafetyEnabled(false);
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
    drive.arcadeDrive(-stick.getRawAxis(1), stick.getRawAxis(2));
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    leftMaster.set(stick.getRawAxis(1));

    // rightSlave.set(stick.getRawAxis(1));
   // shooter.set(Math.pow(stick.getRawAxis(5), 2));
    //shooterFeeder.set(stick.getRawAxis(5));
  }

  public void robotPeriodic() {
    SmartDashboard.putNumber("Shooter Current", shooter.getOutputCurrent());
    SmartDashboard.putNumber("Shooter RPM", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("Left Master Output", leftMaster.getAppliedOutput());
    SmartDashboard.putNumber("LeftSlave Output", leftSlave.getAppliedOutput());
    SmartDashboard.putNumber("Joystick", stick.getRawAxis(1));
  }

}
