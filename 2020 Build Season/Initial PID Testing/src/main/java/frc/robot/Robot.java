/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  
  private CANSparkMax leftMaster, rightMaster, leftSlave, rightSlave;

    private Encoder leftDrive;
    private Encoder rightDrive;

  private boolean kLeftMotorsInverted = true;
  private boolean kRightMotorsInverted = true;

  private DifferentialDrive drive;

  private Joystick stick;
  
  @Override
  public void robotInit() {
    leftMaster = new CANSparkMax(3, MotorType.kBrushless); //3, 2, 6, 5
    leftSlave = new CANSparkMax(2, MotorType.kBrushless);
    rightMaster = new CANSparkMax(6, MotorType.kBrushless);
    rightSlave = new CANSparkMax(5, MotorType.kBrushless);

    leftDrive = new Encoder(7, 6);
    rightDrive = new Encoder(9, 8);

    stick = new Joystick(0);
    
    drive = new DifferentialDrive(leftMaster, rightMaster);

    leftMaster.setInverted(kLeftMotorsInverted);
    leftSlave.setInverted(kLeftMotorsInverted);
    rightMaster.setInverted(kRightMotorsInverted);
    rightSlave.setInverted(kRightMotorsInverted);

    leftDrive.setDistancePerPulse(12.56/1024);
    rightDrive.setDistancePerPulse(12.56/1024);
    rightDrive.setReverseDirection(true);
    leftDrive.reset();
    rightDrive.reset();

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Left Encoder distance", leftDrive.getDistance());
    SmartDashboard.putNumber("right Encoder distance", rightDrive.getDistance());
    SmartDashboard.putNumber("Left encoder counts", leftDrive.getRaw());
    SmartDashboard.putNumber("right encoder counts", rightDrive.getRaw());
    // TODO: chart applied output vs. distance
    // TODO: chart distance oon ground and put both sides in brake
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
    drive.arcadeDrive(stick.getRawAxis(1), -stick.getRawAxis(2));

    if(stick.getRawButton(1)) {
      leftDrive.reset();
      rightDrive.reset();
    }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
