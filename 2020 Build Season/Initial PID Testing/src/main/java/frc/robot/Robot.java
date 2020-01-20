/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  
  private CANSparkMax leftMaster, rightMaster, leftSlave, rightSlave;

  private final boolean kLeftMotorsInverted = true;
  private final boolean kRightMotorsInverted = true;

  CANPIDController leftController;

  CANEncoder leftAlternateEncoder;

  private DifferentialDrive drive;

  private Joystick stick;

  Timer pidTimer;

  private static double kP = 0.006;
  private static double kI = 0;
  private static double kD = 0;
  private static double kFF = 0;
  private static double kMaxOutput = .4; 
  private static double kMinOutput = -.4;
  private static double kpidSetpoint = 48; // inches forward
  private static double kpidTolerance = 2; // inches
  private double processLeft;
  private double processRight;
  private double outputLeft;
  private double outputRight;
  private double autoRamp = 3; // seconds
  
  @Override
  public void robotInit() {
    leftMaster = new CANSparkMax(3, MotorType.kBrushless); //3, 2, 6, 5
    leftSlave = new CANSparkMax(2, MotorType.kBrushless);
    rightMaster = new CANSparkMax(5, MotorType.kBrushless);
    rightSlave = new CANSparkMax(6, MotorType.kBrushless);

    leftAlternateEncoder = leftMaster.getAlternateEncoder(AlternateEncoderType.kQuadrature, 4096);

    leftController = new CANPIDController(leftMaster);
    leftController.setP(kP, 0);
    leftController.setI(kI, 0);
    leftController.setD(kD, 0);
    leftController.setFF(kFF, 0);
    leftController.setOutputRange(kMinOutput, kMaxOutput, 0);
    leftController.setSmartMotionAllowedClosedLoopError(kpidTolerance, 0);

    stick = new Joystick(0);
    
    drive = new DifferentialDrive(leftMaster, rightMaster);
    drive.setSafetyEnabled(false);

    leftMaster.setInverted(kLeftMotorsInverted);
    leftSlave.setInverted(kLeftMotorsInverted);
    rightMaster.setInverted(kRightMotorsInverted);
    rightSlave.setInverted(kRightMotorsInverted);

    leftController.setFeedbackDevice(leftAlternateEncoder);

    pidTimer = new Timer();

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("left output", outputLeft);
    SmartDashboard.putNumber("right output", outputRight);
    SmartDashboard.putNumber("applied output left", leftMaster.getAppliedOutput());
    SmartDashboard.putNumber("applied output right", rightMaster.getAppliedOutput());
    SmartDashboard.putNumber("process left", processLeft);
    SmartDashboard.putNumber("process right", processRight);
    SmartDashboard.putNumber("pid timer", pidTimer.get());
  }

  @Override
  public void autonomousInit() {
    leftMaster.setInverted(!kLeftMotorsInverted);
    leftSlave.setInverted(!kLeftMotorsInverted);
    leftMaster.setClosedLoopRampRate(autoRamp);
  }

  @Override
  public void autonomousPeriodic() {
    if (stick.getRawButton(1)) {
      pidTimer.start();
      leftController.setReference(processLeft, ControlType.kSmartMotion);

    }

    

  }

  @Override
  public void teleopInit() {
    leftMaster.setInverted(kLeftMotorsInverted);
    leftSlave.setInverted(kLeftMotorsInverted);
  }

  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(stick.getRawAxis(1), -stick.getRawAxis(2));
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
