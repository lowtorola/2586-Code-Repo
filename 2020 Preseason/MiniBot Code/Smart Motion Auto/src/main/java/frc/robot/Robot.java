/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {

  DifferentialDrive myRobot;
  AHRS ahrs;
  PIDController turnController;
  double rotateToAngleRate;
  private CANSparkMax leftMaster, leftSlave, rightMaster, rightSlave;
  private CANPIDController left_pidController, right_pidController;
  private CANEncoder left_encoder, right_encoder;
  public double sm_kP, sm_kI, sm_kD, sm_kIz, sm_kFF, sm_kMaxOutput, sm_kMinOutput, sm_maxRPM, sm_maxVel, sm_minVel,
      sm_maxAcc, sm_allowedErr;
  Joystick joystick;
  double setPoint_L, setPoint_R, processVariable_L, processVariable_R;
  final double kInchesPerRevolution = 12.56;
  final double gear_Ratio = 4.16;

  // Turn coefficients
  static final double kP = 0.0275;
  static final double kI = 0.000035;
  static final double kD = 0.115;
  static final double kF = 0;

  static final double kToleranceDegrees = 2.0f;

  static final double kTargetAngleDegrees = 0.0f;

  @Override
  public void robotInit() {

    joystick = new Joystick(0);
    // initialize motor
    boolean leftMotorsInverted = false;
    boolean rightMotorsInverted = true;

    leftMaster = new CANSparkMax(3, MotorType.kBrushless);
    leftMaster.setInverted(leftMotorsInverted);
    leftSlave = new CANSparkMax(2, MotorType.kBrushless);
    leftSlave.setInverted(leftMotorsInverted);
    leftSlave.follow(leftMaster);
    rightMaster = new CANSparkMax(6, MotorType.kBrushless);
    rightMaster.setInverted(rightMotorsInverted);
    rightSlave = new CANSparkMax(5, MotorType.kBrushless);
    rightSlave.setInverted(rightMotorsInverted);
    rightSlave.follow(rightMaster);

    ahrs = new AHRS(SPI.Port.kMXP);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration
     * parameters in the SPARK MAX to their factory default state. If no argument is
     * passed, these parameters will not persist between power cycles
     */
    leftMaster.setSmartCurrentLimit(40);
    leftSlave.setSmartCurrentLimit(40);
    rightMaster.setSmartCurrentLimit(40);
    rightSlave.setSmartCurrentLimit(40);

    // initialze PID controller and encoder objects
    left_pidController = leftMaster.getPIDController();
    right_pidController = rightMaster.getPIDController();
    left_encoder = leftMaster.getEncoder();
    right_encoder = rightMaster.getEncoder();
    left_encoder.setPosition(0);
    right_encoder.setPosition(0);

    // PID coefficients
    sm_kP = 3e-5;
    sm_kI = 3e-7;
    sm_kD = 0;
    sm_kIz = 0;
    sm_kFF = 0.00002;
    sm_kMaxOutput = 0.5;
    sm_kMinOutput = -0.5;
    sm_maxRPM = 5700;
    sm_allowedErr = 2;

    // Smart Motion Coefficients
    sm_maxVel = 2000; // rpm
    sm_maxAcc = 1200;

    // set PID coefficients
    left_pidController.setP(sm_kP);
    left_pidController.setI(sm_kI);
    left_pidController.setD(sm_kD);
    left_pidController.setIZone(sm_kIz);
    left_pidController.setFF(sm_kFF);
    left_pidController.setOutputRange(sm_kMinOutput, sm_kMaxOutput);

    right_pidController.setP(sm_kP);
    right_pidController.setI(sm_kI);
    right_pidController.setD(sm_kD);
    right_pidController.setIZone(sm_kIz);
    right_pidController.setFF(sm_kFF);
    right_pidController.setOutputRange(sm_kMinOutput, sm_kMaxOutput);

    /**
     * Smart Motion coefficients are set on a CANPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of the pid
     * controller in Smart Motion mode - setSmartMotionMinOutputVelocity() will put
     * a lower bound in RPM of the pid controller in Smart Motion mode -
     * setSmartMotionMaxAccel() will limit the acceleration in RPM^2 of the pid
     * controller in Smart Motion mode - setSmartMotionAllowedClosedLoopError() will
     * set the max allowed error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot_L = 0;
    left_pidController.setSmartMotionMaxVelocity(sm_maxVel, smartMotionSlot_L);
    left_pidController.setSmartMotionMinOutputVelocity(sm_minVel, smartMotionSlot_L);
    left_pidController.setSmartMotionMaxAccel(sm_maxAcc, smartMotionSlot_L);
    left_pidController.setSmartMotionAllowedClosedLoopError(sm_allowedErr, smartMotionSlot_L);

    left_pidController.setOutputRange(sm_kMinOutput, sm_kMaxOutput);
    left_pidController.setP(sm_kP);
    left_pidController.setI(sm_kI);
    left_pidController.setD(sm_kD);
    left_pidController.setIZone(sm_kIz);
    left_pidController.setFF(sm_kFF);
    left_pidController.setSmartMotionMaxVelocity(sm_maxVel, 0);
    left_pidController.setSmartMotionMinOutputVelocity(sm_minVel, 0);
    left_pidController.setSmartMotionMaxAccel(sm_maxAcc, 0);
    left_pidController.setSmartMotionAllowedClosedLoopError(sm_allowedErr, 0);

    int smartMotionSlot_R = 1;
    right_pidController.setSmartMotionMaxVelocity(sm_maxVel, smartMotionSlot_R);
    right_pidController.setSmartMotionMinOutputVelocity(sm_minVel, smartMotionSlot_R);
    right_pidController.setSmartMotionMaxAccel(sm_maxAcc, smartMotionSlot_R);
    right_pidController.setSmartMotionAllowedClosedLoopError(sm_allowedErr, smartMotionSlot_R);
    right_pidController.setOutputRange(sm_kMinOutput, sm_kMaxOutput);
    right_pidController.setP(sm_kP);
    right_pidController.setI(sm_kI);
    right_pidController.setD(sm_kD);
    right_pidController.setIZone(sm_kIz);
    right_pidController.setFF(sm_kFF);
    right_pidController.setSmartMotionMaxVelocity(sm_maxVel, 0);
    right_pidController.setSmartMotionMinOutputVelocity(sm_minVel, 0);
    right_pidController.setSmartMotionMaxAccel(sm_maxAcc, 0);
    right_pidController.setSmartMotionAllowedClosedLoopError(sm_allowedErr, 0);

    setPoint_L = neoDistConv(20);
    setPoint_R = neoDistConv(20);

    turnController = new PIDController(kP, kI, kD, ahrs, this);
    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-0.5, 0.5);
    turnController.setAbsoluteTolerance(kToleranceDegrees);
    turnController.setContinuous(true);
    turnController.disable();

    myRobot = new DifferentialDrive(leftMaster, rightMaster);
    myRobot.setExpiration(0.1);

  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    left_pidController.setReference(setPoint_L, ControlType.kSmartMotion);
    right_pidController.setReference(setPoint_R, ControlType.kSmartMotion);
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {

    sensorPrints();

    if (joystick.getRawButton(1)) {
      left_pidController.setReference(setPoint_L, ControlType.kSmartMotion);
      right_pidController.setReference(setPoint_R, ControlType.kSmartMotion);
    } else {
      leftMaster.set(0);
      rightMaster.set(0);
    }
    processVariable_L = left_encoder.getPosition();
    processVariable_R = right_encoder.getPosition();

    if (joystick.getRawButton(2)) {
      zeroEncoders();
    }

  }

  @Override
  public void testInit() {
  }

  public void zeroEncoders() {
    left_encoder.setPosition(0);
    right_encoder.setPosition(0);
  }

  public void sensorPrints() {
    SmartDashboard.putNumber("SetPoint L", setPoint_L);
    SmartDashboard.putNumber("Setpoint R", setPoint_R);
    SmartDashboard.putNumber("Process Variable Left", processVariable_L);
    SmartDashboard.putNumber("Process Variable Right", processVariable_R);
    SmartDashboard.putNumber("Left Output", leftMaster.getAppliedOutput());
    SmartDashboard.putNumber("Right Output", rightMaster.getAppliedOutput());
  }

  @Override
  public void testPeriodic() {
  }

  public double neoDistConv(double x) {
    x /= gear_Ratio;
    x *= kInchesPerRevolution;
    return x;
  }

  public void autonDriveTurn() {
    left_pidController.setReference(setPoint_L, ControlType.kSmartMotion);
    right_pidController.setReference(setPoint_R, ControlType.kSmartMotion);

  }

}
