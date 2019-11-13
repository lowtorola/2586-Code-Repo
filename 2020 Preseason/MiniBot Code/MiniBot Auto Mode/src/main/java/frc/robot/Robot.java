/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
//  DifferentialDrive m_drive;

  Joystick m_joystick = new Joystick(0);
  double drive_speed;
  double drive_rotate;

  CANSparkMax leftMaster;
  CANSparkMax leftSlave;
  CANSparkMax rightMaster;
  CANSparkMax rightSlave;

  private CANEncoder f_leftEncoder;
  private CANEncoder f_rightEncoder;

  CANPIDController autoPIDdrive_L;
  CANPIDController autoPIDdrive_R;

  double setPoint, processVariable;

  AHRS gyro;
  double gyroYaw;
  double gyroVelocity;

  Compressor comp;

  DoubleSolenoid shifter;

  @Override
  public void robotInit() {

    boolean leftMotorsInverted = false;
    boolean rightMotorsInverted = true;

    leftMaster = new CANSparkMax(3, MotorType.kBrushless);
    leftMaster.setInverted(leftMotorsInverted);
    leftSlave = new CANSparkMax(2, MotorType.kBrushless);
    leftSlave.setInverted(leftMotorsInverted);
    rightMaster = new CANSparkMax(6, MotorType.kBrushless);
    rightMaster.setInverted(rightMotorsInverted);
    rightSlave = new CANSparkMax(5, MotorType.kBrushless);
    rightSlave.setInverted(rightMotorsInverted);

    leftMaster.setIdleMode(IdleMode.kBrake);
    leftSlave.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave.setIdleMode(IdleMode.kBrake);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    f_leftEncoder = leftMaster.getEncoder();
    f_rightEncoder = rightMaster.getEncoder();

    autoPIDdrive_L = leftMaster.getPIDController();
   // autoPIDdrive_R = rightMaster.getPIDController();

    autoPIDdrive_L.setP(Constants.kP);
    autoPIDdrive_L.setI(Constants.kI);
    autoPIDdrive_L.setD(Constants.kD);
    autoPIDdrive_L.setIZone(Constants.kIz);
    autoPIDdrive_L.setFF(Constants.kFF);
    autoPIDdrive_L.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);

    /*
    autoPIDdrive_R.setP(Constants.kP);
    autoPIDdrive_R.setI(Constants.kI);
    autoPIDdrive_R.setD(Constants.kD);
    autoPIDdrive_R.setIZone(Constants.kIz);
    autoPIDdrive_R.setFF(Constants.kFF);
    autoPIDdrive_R.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);
*/
    //autoPIDdrive_L.setSmartMotionMinOutputVelocity(Constants.minVel, Constants.smartMotionSlot_L);
    autoPIDdrive_L.setSmartMotionMaxAccel(Constants.maxAcc, Constants.smartMotionSlot_L);
    autoPIDdrive_L.setSmartMotionAllowedClosedLoopError(Constants.allowedErr, Constants.smartMotionSlot_L);    
    //autoPIDdrive_L.setSmartMotionMaxVelocity(Constants.maxVel, Constants.smartMotionSlot_L);
/*
    autoPIDdrive_R.setSmartMotionMinOutputVelocity(Constants.minVel, Constants.smartMotionSlot_R);
    autoPIDdrive_R.setSmartMotionMaxAccel(Constants.maxAcc, Constants.smartMotionSlot_R);
    autoPIDdrive_R.setSmartMotionAllowedClosedLoopError(Constants.allowedErr, Constants.smartMotionSlot_R);   
    autoPIDdrive_R.setSmartMotionMaxVelocity(Constants.maxVel, Constants.smartMotionSlot_R);
   */
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();

    //comp = new Compressor();
   // comp.start();

    shifter = new DoubleSolenoid(6, 7);

  //  m_drive = new DifferentialDrive(leftMaster, rightMaster);

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
    
    setPoint = 100;
    
    if (m_joystick.getRawButton(1)) {
      autoPIDdrive_L.setReference(setPoint, ControlType.kSmartMotion);
      //autoPIDdrive_R.setReference(setPoint, ControlType.kSmartMotion);
      rightMaster.follow(leftMaster);
    } else {
      leftMaster.set(0);
      rightMaster.set(0);
    }

    if(m_joystick.getRawButton(2)) {
      f_leftEncoder.setPosition(0);
      f_rightEncoder.setPosition(0);
    }

    sensorPrints();
    
  }

  public void sensorPrints() {
    processVariable = f_leftEncoder.getPosition();

    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Left Output", leftMaster.getAppliedOutput());
    SmartDashboard.putNumber("Right Output", rightMaster.getAppliedOutput());

  }

  @Override
  public void disabledInit() {
    super.disabledInit();
    leftMaster.setIdleMode(IdleMode.kCoast);
    leftSlave.setIdleMode(IdleMode.kCoast);
    rightMaster.setIdleMode(IdleMode.kCoast);
    rightSlave.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
