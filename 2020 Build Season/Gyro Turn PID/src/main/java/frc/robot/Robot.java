/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint.MinMax;
import edu.wpi.first.wpiutil.math.MathUtil;


public class Robot extends TimedRobot {
  
  private CANSparkMax leftMaster, rightMaster, leftSlave, rightSlave;

  private Encoder leftDrive;
  private Encoder rightDrive;

  AHRS ahrs;


  private PIDController gyroTurnController;
  private double currentAngle;
  private double pidTurnOutput;
  final double TARGET_ANGLE = -65;
  final double DEADBAND = 0.2;
  final double MAX_OUTPUT = 0.4;
  final double MIN_OUTPUT = -0.4;
  private boolean isAtAngle;

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

    ahrs = new AHRS(SPI.Port.kMXP);

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

    final double iP = 0.05;
    final double iI = 0.0;
    final double iD = 0.001;

    gyroTurnController = new PIDController(iP, iI, iD);
    gyroTurnController.setTolerance(2.5);
    gyroTurnController.enableContinuousInput(-180, 180);

    aimTimer = new Timer();

    leftMaster.setIdleMode(IdleMode.kBrake);
    leftSlave.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("pid Output X", pidTurnOutput);
    SmartDashboard.putNumber("Aiming Time", aimTimer.get());
    SmartDashboard.putNumber("gyro angle", ahrs.getAngle());
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    aimTimer.reset();
    ahrs.reset();
  }

  @Override
  public void teleopPeriodic() {
    
    if (stick.getRawButton(6)) {

      pidTurnOutput = MathUtil.clamp(gyroTurnController.calculate(ahrs.getAngle(), TARGET_ANGLE), MIN_OUTPUT, MAX_OUTPUT);

      drive.arcadeDrive(0, -pidTurnOutput);

    } else { 

      gyroTurnController.reset();

      drive.arcadeDrive(stick.getRawAxis(1), -stick.getRawAxis(2), true);
    }

    if(stick.getRawButton(1)) {
      leftDrive.reset();
      rightDrive.reset();
      ahrs.reset();
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