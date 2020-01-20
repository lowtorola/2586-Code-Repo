/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Robot extends TimedRobot {
  
  private CANSparkMax leftMaster, rightMaster, leftSlave, rightSlave;

  private Encoder leftDrive;
  private Encoder rightDrive;

  private PIDController limeLightController;
  private double tx, ty;
  private double pidOutput;
  final double TARGET_ANGLE = 0;
  final double DEADBAND = 0.2;
  final double MAX_OUTPUT = 0.4;
  final double MIN_OUTPUT = -0.4;

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

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

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

    final double iP = 0.036;
    // final double iI = 0.000035;
    final double iI = 0.000001;
    // final double iD = 0.115;
    final double iD = 0;

    limeLightController.setPID(iP, iI, iD);


  }

  @Override
  public void robotPeriodic() {
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
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
    
    if (stick.getRawButton(6)) {
      leftMaster.setIdleMode(IdleMode.kCoast);
      leftSlave.setIdleMode(IdleMode.kCoast);
      rightMaster.setIdleMode(IdleMode.kCoast);
      rightSlave.setIdleMode(IdleMode.kCoast);

      pidOutput = limeLightController.calculate(tx, TARGET_ANGLE);

      drive.arcadeDrive(0, pidOutput);
    } else { 
      leftMaster.setIdleMode(IdleMode.kBrake);
      leftSlave.setIdleMode(IdleMode.kBrake);
      rightMaster.setIdleMode(IdleMode.kBrake);
      rightSlave.setIdleMode(IdleMode.kBrake);

      drive.arcadeDrive(stick.getRawAxis(1), -stick.getRawAxis(2), true);
    }

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