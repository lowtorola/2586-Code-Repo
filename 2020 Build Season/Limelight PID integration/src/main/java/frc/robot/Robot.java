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

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;


public class Robot extends TimedRobot {
  
  private CANSparkMax leftMaster, rightMaster, leftSlave, rightSlave;

  private Encoder leftDrive;
  private Encoder rightDrive;

  private PIDController limeLightControllerX, limeLightControllerY;
  private double tx, ty, tv;
  private double pidOutputX;
  private double pidOutputY;
  final double TARGET_ANGLE_X = 0;
  final double TARGET_ANGLE_Y = 0;
  final double DEADBAND = 0.2;
  final double MAX_OUTPUT = 0.4;
  final double MIN_OUTPUT = -0.4;
  private boolean isOnTarget = false;
  private boolean targetAcquired = false;
  private boolean isInRange = false;

  private boolean kLeftMotorsInverted = true;
  private boolean kRightMotorsInverted = true;

  private DifferentialDrive drive;

  private Joystick stick;

  private Timer aimTimer;
  
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

    final double iP = 0.03;
    final double iI = 0.08;
    final double iD = 0;

    limeLightControllerX = new PIDController(iP, iI, iD);
    limeLightControllerX.setTolerance(2.5);

    limeLightControllerY = new PIDController(iP, iI, iD);
    limeLightControllerY.setTolerance(2.5);

    aimTimer = new Timer();

  }

  @Override
  public void robotPeriodic() {
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putBoolean("Target Acquired", targetAcquired);
    SmartDashboard.putBoolean("On Target", isOnTarget);
    SmartDashboard.putBoolean("In Range", isInRange);
    SmartDashboard.putNumber("pid Output X", pidOutputX);
    SmartDashboard.putNumber("pid Output Y", pidOutputY);
    SmartDashboard.putNumber("Aiming Time", aimTimer.get());
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    aimTimer.reset();
  }

  @Override
  public void teleopPeriodic() {

    ledCheck();
    targetCheck();
    alignmentCheck();
    distCheck();

    if (stick.getRawButton(6)) {
      aimTimeCalc();
    }
    
    if (stick.getRawButton(6)) {

      leftMaster.setIdleMode(IdleMode.kCoast);
      leftSlave.setIdleMode(IdleMode.kCoast);
      rightMaster.setIdleMode(IdleMode.kCoast);
      rightSlave.setIdleMode(IdleMode.kCoast);

      pidOutputX = MathUtil.clamp(limeLightControllerX.calculate(tx, TARGET_ANGLE_X), -0.4, 0.4);
      pidOutputY = MathUtil.clamp(limeLightControllerY.calculate(ty, TARGET_ANGLE_Y), -0.4, 0.4);

      drive.arcadeDrive(-pidOutputY, pidOutputX);

    } else { 

      leftMaster.setIdleMode(IdleMode.kBrake);
      leftSlave.setIdleMode(IdleMode.kBrake);
      rightMaster.setIdleMode(IdleMode.kBrake);
      rightSlave.setIdleMode(IdleMode.kBrake);

      limeLightControllerX.reset();
      limeLightControllerY.reset();

      drive.arcadeDrive(stick.getRawAxis(1), -stick.getRawAxis(2), true);
    }

    if(stick.getRawButton(1)) {
      leftDrive.reset();
      rightDrive.reset();
    }
  }

  public void ledCheck() {
    if(stick.getRawButton(6)) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    }
  }

  public void targetCheck() {
    if (tv == 1) {
      targetAcquired = true;
    } else {
      targetAcquired = false;
    }
  }

  public void alignmentCheck() {
    if (tx == 0.0) {
      isOnTarget = false;
    } else if (Math.abs(tx) < 3) {
      isOnTarget = true;
    } else {
      isOnTarget = false;
    }
  }

  public void distCheck() {
    if (ty == 0.0) {
      isInRange = false;
    } else if (Math.abs(ty) < 5) {
      isInRange = true;
    } else {
      isInRange = false;
    }
  }

  public void aimTimeCalc() {
    aimTimer.start();
    if (targetAcquired && isOnTarget && isInRange) {
      aimTimer.stop();
    }
  }

  public void disabledPeriodic() {
    aimTimer.stop();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}