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
  private double leftDist;
  private double rightDist;

  private boolean kLeftMotorsInverted = true;
  private boolean kRightMotorsInverted = true;

  private DifferentialDrive drive;

  private PIDController gyroTurnController;
  private double currentAngle;
  private double gyro_pidTurnOutput;
  final double GYRO_TARGET_ANGLE = -90;
  final double GYRO_DEADBAND = 0.2;
  final double GYRO_MAX_OUTPUT = 0.3;
  final double GYRO_MIN_OUTPUT = -0.3;
  private boolean gyro_isAtAngle;

  PIDController autoDriveControllerL;
  PIDController autoDriveControllerR;
  private double leftAutoOutput;
  private double rightAutoOutput;
  final double DT_MAX_OUTPUT = 0.4;
  final double DT_MIN_OUTPUT = -0.4;
  final double DT_TARGET_DIST = 24; // inches

  private Joystick stick;

  private Timer autoTimer;

  private AHRS gyro;

  int autoStep;
  double DT_error_left;
  double DT_error_right;
  double GYRO_error;

  @Override
  public void robotInit() {
    leftMaster = new CANSparkMax(3, MotorType.kBrushless); // 3, 2, 6, 5
    leftSlave = new CANSparkMax(2, MotorType.kBrushless);
    rightMaster = new CANSparkMax(6, MotorType.kBrushless);
    rightSlave = new CANSparkMax(5, MotorType.kBrushless);

    leftDrive = new Encoder(7, 6);
    rightDrive = new Encoder(9, 8);

    stick = new Joystick(0);

    gyro = new AHRS(SPI.Port.kMXP);

    autoTimer = new Timer();

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(kLeftMotorsInverted);
    leftSlave.setInverted(kLeftMotorsInverted);
    rightMaster.setInverted(kRightMotorsInverted);
    rightSlave.setInverted(kRightMotorsInverted);

    leftMaster.setIdleMode(IdleMode.kBrake);
    leftSlave.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave.setIdleMode(IdleMode.kBrake);

    leftDrive.setDistancePerPulse(12.56 / 1024);
    rightDrive.setDistancePerPulse(12.56 / 1024);
    rightDrive.setReverseDirection(true);
    leftDrive.reset();
    rightDrive.reset();

    final double GYRO_iP = 0.05;
    final double GYRO_iI = 0.0;
    final double GYRO_iD = 0.001;

    gyroTurnController = new PIDController(GYRO_iP, GYRO_iI, GYRO_iD);
    gyroTurnController.setTolerance(2.5);
    gyroTurnController.enableContinuousInput(-180, 180);

    final double DTkP = 0.05;
    final double DTkI = 0.0;
    final double DTkD = 0;

    final double DT_PID_TOLERANCE = 0.2;
    autoDriveControllerL = new PIDController(DTkP, DTkI, DTkD);
    autoDriveControllerR = new PIDController(DTkP, DTkI, DTkD);

    autoDriveControllerL.setTolerance(DT_PID_TOLERANCE);
    autoDriveControllerR.setTolerance(DT_PID_TOLERANCE);

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
    SmartDashboard.putNumber("pid Output X", gyro_pidTurnOutput);
    //SmartDashboard.putNumber("Aiming Time", autoTimer.get());
    SmartDashboard.putNumber("gyro angle", gyro.getAngle());
    SmartDashboard.putNumber("auto step", autoStep);
    SmartDashboard.putNumber("error left", DT_error_left);
    SmartDashboard.putNumber("turn error", GYRO_error);
  }

  @Override
  public void autonomousInit() {
    autoStep = 1;
    resetAll();
  }

  @Override
  public void autonomousPeriodic() {
    encDistCheck();
    DT_error_left = Math.abs(DT_TARGET_DIST - leftDist);
    DT_error_right = Math.abs(DT_TARGET_DIST - rightDist);
    GYRO_error = Math.abs(GYRO_TARGET_ANGLE - currentAngle);
    switch (autoStep) {
      case 1:
      if (DT_error_left > 1 && DT_error_right > 1) {
        autoDriveForward();
      } else {
        autoNextStep();
      }
      break;

      case 2:
      if (GYRO_error > 2) {
        autoTurn();
      } else {
        autoNextStep();
      }
      break;

      case 3:
      if (DT_error_left > 1 && DT_error_right > 1) {
        autoDriveForward();
      } else {
        autoNextStep();
      }
      break;

      case 4:
      if (GYRO_error > 2) {
        autoTurn();
      } else {
        autoNextStep();
      }
      break;

      case 5:
      if (DT_error_left > 1 && DT_error_right > 1) {
        autoDriveForward();
      } else {
        autoNextStep();
      }
      break;

      case 6:
      if (GYRO_error > 2) {
        autoTurn();
      } else {
        autoNextStep();
      }
      break;

      case 7:
      if (DT_error_left > 1 && DT_error_right > 1) {
        autoDriveForward();
      } else {
        autoNextStep();
      }
      break;

      case 8:
      if (GYRO_error > 2) {
        autoTurn();
      } else {
        autoNextStep();
      }
      break;

      default:
      drive.arcadeDrive(0, 0);
      break;
    }
   }

  @Override
  public void teleopInit() {
    autoTimer.reset();
    gyro.reset();
  }

  @Override
  public void teleopPeriodic() {
    if(stick.getRawButton(4)) {
      resetAll();
    }
    encDistCheck();

    if (stick.getRawButton(1)) {
      autoDriveForward();
    } else if (stick.getRawButton(2)) {
      autoTurn();
    } else {
      drive.arcadeDrive(stick.getRawAxis(1), -stick.getRawAxis(2));
    }
  }

  public void encDistCheck() {
    leftDist = leftDrive.getDistance();
    rightDist = rightDrive.getDistance();
    currentAngle = gyro.getAngle();
  }

  public void autoDriveForward() {

    leftAutoOutput = MathUtil.clamp(autoDriveControllerL.calculate(leftDist, DT_TARGET_DIST), DT_MIN_OUTPUT,
        DT_MAX_OUTPUT);
    rightAutoOutput = MathUtil.clamp(autoDriveControllerR.calculate(rightDist, DT_TARGET_DIST), DT_MIN_OUTPUT,
        DT_MAX_OUTPUT);

    drive.tankDrive(-leftAutoOutput, -rightAutoOutput);

  }

  public void autoTurn() {

    gyro_pidTurnOutput = MathUtil.clamp(gyroTurnController.calculate(currentAngle, GYRO_TARGET_ANGLE),
        GYRO_MIN_OUTPUT, GYRO_MAX_OUTPUT);

    drive.arcadeDrive(0, -gyro_pidTurnOutput);

  }

  public void autoNextStep() {
    resetAll();
    autoStep++;
  }

  public void resetAll() {
    
      gyro.reset();
      leftDrive.reset();
      rightDrive.reset();
    
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}