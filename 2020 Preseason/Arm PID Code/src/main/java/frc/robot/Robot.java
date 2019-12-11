/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  Joystick joystick;
  CANSparkMax armMotor;
  CANEncoder motor_Encoder;
  CANPIDController armController;
  double armControl;
  boolean armLimitReverse, armLimitForward;
  CANDigitalInput m_reverseLimit;
  CANDigitalInput m_forwardLimit;
  double sm_kP, sm_kI, sm_kD, sm_kIz, sm_kFF, sm_kMaxOutput, sm_kMinOutput, sm_maxRPM, sm_maxVel, sm_minVel, sm_maxAcc,
      sm_allowedErr;
  double setPointHigh, setPointLow;
  int setPointSelector = 0;
  boolean isPID_Enabled = false;

  @Override
  public void robotInit() {
    joystick = new Joystick(0);
    armMotor = new CANSparkMax(4, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setInverted(false);
    motor_Encoder = armMotor.getEncoder();
    armController = armMotor.getPIDController();
    m_reverseLimit = armMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_forwardLimit = armMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_reverseLimit.enableLimitSwitch(true);
    m_forwardLimit.enableLimitSwitch(true);

    setPointHigh = 90;
    setPointLow = 20;

    sm_kP = 0;
    sm_kI = 0;
    sm_kD = 0;
    sm_kIz = 0;
    ;
    sm_kMaxOutput = 1;
    sm_kMinOutput = -1;
    sm_maxRPM = 5700;
    sm_allowedErr = 5;

    armController.setP(sm_kP);
    armController.setI(sm_kI);
    armController.setD(sm_kD);
    armController.setIZone(sm_kIz);
    armController.setFF(sm_kFF);
    armController.setOutputRange(sm_kMinOutput, sm_kMaxOutput);

    armController.setP(sm_kP);
    armController.setI(sm_kI);
    armController.setD(sm_kD);
    armController.setIZone(sm_kIz);
    armController.setFF(sm_kFF);
    armController.setOutputRange(sm_kMinOutput, sm_kMaxOutput);

    int smartMotionSlot_L = 1;
    armController.setSmartMotionMaxVelocity(sm_maxVel, smartMotionSlot_L);
    armController.setSmartMotionMinOutputVelocity(sm_minVel, smartMotionSlot_L);
    armController.setSmartMotionMaxAccel(sm_maxAcc, smartMotionSlot_L);
    armController.setSmartMotionAllowedClosedLoopError(sm_allowedErr, smartMotionSlot_L);

    armController.setOutputRange(sm_kMinOutput, sm_kMaxOutput);
    armController.setP(sm_kP);
    armController.setI(sm_kI);
    armController.setD(sm_kD);
    armController.setIZone(sm_kIz);
    armController.setFF(sm_kFF);
    armController.setSmartMotionMaxVelocity(sm_maxVel, 0);
    armController.setSmartMotionMinOutputVelocity(sm_minVel, 0);
    armController.setSmartMotionMaxAccel(sm_maxAcc, 0);
    armController.setSmartMotionAllowedClosedLoopError(sm_allowedErr, 0);

    motor_Encoder.setPosition(0);
    SmartDashboard.putBoolean("Toggle PID", false);

  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("Reverse Limit", m_reverseLimit.get());
    SmartDashboard.putBoolean("Forward Limit", m_forwardLimit.get());
    SmartDashboard.putNumber("Arm Motor Input", armControl);
    SmartDashboard.putNumber("Arm Encoder Rotations", motor_Encoder.getPosition());
    SmartDashboard.putNumber("Arm Motor Output", armMotor.getAppliedOutput());
    SmartDashboard.putNumber("setpoint selector", setPointSelector);
    isPID_Enabled = SmartDashboard.getBoolean("Toggle PID", false);
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    positionSelector();
    armLimitReverse = m_reverseLimit.get();
    armLimitForward = m_forwardLimit.get();
    if (armLimitReverse) {
      motor_Encoder.setPosition(0);
    }
    if (isPID_Enabled) {
      System.out.println("PID is Enabled");
      switch (setPointSelector) {
      case 1:
        if (joystick.getRawButton(2)) {
          armController.setReference(setPointLow, ControlType.kPosition);
        }
        break;
      case 2:
        if (joystick.getRawButton(2)) {
          armController.setReference(setPointHigh, ControlType.kPosition);
        }
        break;
      }
    } else if (!isPID_Enabled) {
      System.out.println("PID is disabled");
      armMotor.set(deadZone(joystick.getRawAxis(1)));
    } else {
      armMotor.set(0);
      System.out.println("Arm is stopped");
    }

  }

  public double deadZone(double x) {
    if (Math.abs(x) < 0.04) {
      return 0;
    } else {
      return x;
    }
  }

  public void positionSelector() {
    if (joystick.getRawButton(1)) {
      setPointSelector = 1;
    } else if (joystick.getRawButton(2)) {
      setPointSelector = 2;
    }
  }

  public void controlModeConverter() {

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
