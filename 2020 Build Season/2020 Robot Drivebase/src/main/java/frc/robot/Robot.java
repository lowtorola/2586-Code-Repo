/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class Robot extends TimedRobot {
 
  CANSparkMax leftMaster, leftSlave, rightMaster, rightSlave;

  Joystick stick;

  DifferentialDrive drive;
  
  @Override
  public void robotInit() {
    // left inverted?
    leftMaster = new CANSparkMax(1, MotorType.kBrushless);
    leftSlave = new CANSparkMax(2, MotorType.kBrushless);
    rightMaster = new CANSparkMax(3, MotorType.kBrushless);
    rightSlave = new CANSparkMax(4, MotorType.kBrushless);

    stick = new Joystick(0);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    leftMaster.setInverted(false);
    rightMaster.setInverted(false);

    drive = new DifferentialDrive(leftMaster, rightMaster);
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
    // rightSlave.set(stick.getRawAxis(1));
  }

}
