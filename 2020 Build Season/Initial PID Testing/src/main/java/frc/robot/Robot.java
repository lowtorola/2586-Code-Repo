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
import edu.wpi.first.wpiutil.math.MathUtil;


public class Robot extends TimedRobot {

  private CANSparkMax leftMaster, rightMaster, leftSlave, rightSlave;

    private Encoder leftDrive;
    private Encoder rightDrive;
    private double leftDist;
    private double rightDist;

  private boolean kLeftMotorsInverted = true;
  private boolean kRightMotorsInverted = true;

  private DifferentialDrive drive;

  PIDController autoDriveControllerL;
  PIDController autoDriveControllerR;
  private double leftAutoOutput;
  private double rightAutoOutput;
  final double MAX_OUTPUT = 0.4;
  final double MIN_OUTPUT = -0.4;
  final double TARGET_DIST = 36; // inches


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

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(kLeftMotorsInverted);
    leftSlave.setInverted(kLeftMotorsInverted);
    rightMaster.setInverted(kRightMotorsInverted);
    rightSlave.setInverted(kRightMotorsInverted);

    leftDrive.setDistancePerPulse(12.56/1024);
    rightDrive.setDistancePerPulse(12.56/1024);
    rightDrive.setReverseDirection(true);
    leftDrive.reset();
    rightDrive.reset();

    final double kP = 0.008;
    final double kI = 0.02;
    final double kD = 0;

    final double PID_TOLERANCE = 0.2;
    autoDriveControllerL = new PIDController(kP, kI, kD);
    autoDriveControllerR = new PIDController(kP, kI, kD);

    autoDriveControllerL.setTolerance(PID_TOLERANCE);
    autoDriveControllerR.setTolerance(PID_TOLERANCE);

    drive = new DifferentialDrive(leftMaster, rightMaster);
    drive.setSafetyEnabled(false);

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Left Encoder distance", leftDrive.getDistance());
    SmartDashboard.putNumber("right Encoder distance", rightDrive.getDistance());
    SmartDashboard.putNumber("Left encoder counts", leftDrive.getRaw());
    SmartDashboard.putNumber("right encoder counts", rightDrive.getRaw());
    SmartDashboard.putNumber("left output", leftAutoOutput);
    SmartDashboard.putNumber("right output", rightAutoOutput);
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

    encDistCheck();
    if (stick.getRawButton(1)) {
      leftMaster.setIdleMode(IdleMode.kBrake);
      leftSlave.setIdleMode(IdleMode.kBrake);
      rightMaster.setIdleMode(IdleMode.kBrake);
      rightSlave.setIdleMode(IdleMode.kBrake);

      leftAutoOutput = MathUtil.clamp(autoDriveControllerL.calculate(leftDist, TARGET_DIST), MIN_OUTPUT, MAX_OUTPUT);
      rightAutoOutput = MathUtil.clamp(autoDriveControllerR.calculate(rightDist, TARGET_DIST), MIN_OUTPUT, MAX_OUTPUT);

      drive.tankDrive(-leftAutoOutput, -rightAutoOutput);

    } else {
      drive.arcadeDrive(stick.getRawAxis(1), -stick.getRawAxis(2));
    }
    if(stick.getRawButton(4)) {
      leftDrive.reset();
      rightDrive.reset();
    }
  }

  public void encDistCheck() {
    leftDist = leftDrive.getDistance();
    rightDist = rightDrive.getDistance();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}