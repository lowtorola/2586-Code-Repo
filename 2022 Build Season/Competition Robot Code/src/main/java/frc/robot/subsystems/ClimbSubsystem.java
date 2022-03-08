// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsystem extends SubsystemBase {

  private final CANSparkMax m_rightTele = new CANSparkMax(RIGHT_TELESCOPE, MotorType.kBrushless);
  private final CANSparkMax m_leftTele = new CANSparkMax(LEFT_TELESCOPE, MotorType.kBrushless);
  private final RelativeEncoder m_leftTeleEnc;
  private final RelativeEncoder m_rightTeleEnc;
  private final DoubleSolenoid m_pivot = new DoubleSolenoid(PneumaticsModuleType.REVPH, PIVOT[0], PIVOT[1]);

  private final SparkMaxPIDController m_leftController;
  private final SparkMaxPIDController m_rightController;
  private final SparkMaxLimitSwitch m_leftLimit;
  private final SparkMaxLimitSwitch m_rightLimit;

  public ClimbSubsystem() {

    m_rightTele.setInverted(true);
    m_leftTele.setSmartCurrentLimit(40);
    m_rightTele.setSmartCurrentLimit(40);

    m_leftTeleEnc = m_leftTele.getEncoder();
    m_rightTeleEnc = m_rightTele.getEncoder();

    resetEncoders();

    m_leftController = m_leftTele.getPIDController();
    m_rightController = m_rightTele.getPIDController();

    m_leftLimit = m_leftTele.getReverseLimitSwitch(Type.kNormallyOpen);
    m_rightLimit = m_rightTele.getReverseLimitSwitch(Type.kNormallyOpen);

    // set left gains
    m_leftController.setP(KP_LEFT);
    m_leftController.setI(KI);
    m_leftController.setD(KD);
    m_leftController.setIZone(KIZ);
    m_leftController.setFF(KFF_LEFT);
    m_leftController.setOutputRange(KMIN_OUTPUT, KMAX_OUTPUT);
    // set right gains
    m_rightController.setP(KP_RIGHT);
    m_rightController.setI(KI);
    m_rightController.setD(KD);
    m_rightController.setIZone(KIZ);
    m_rightController.setFF(KFF_RIGHT);
    m_rightController.setOutputRange(KMIN_OUTPUT, KMAX_OUTPUT);
    // set left smart motion settings
    m_leftController.setSmartMotionMaxVelocity(MAX_VEL, SMART_MOTION_SLOT);
    m_leftController.setSmartMotionMinOutputVelocity(MIN_VEL, SMART_MOTION_SLOT);
    m_leftController.setSmartMotionMaxAccel(MAX_ACC, SMART_MOTION_SLOT);
    m_leftController.setSmartMotionAllowedClosedLoopError(ALLOWED_ERR, SMART_MOTION_SLOT);
    // set right smart motion settings
    m_rightController.setSmartMotionMaxVelocity(MAX_VEL, SMART_MOTION_SLOT);
    m_rightController.setSmartMotionMinOutputVelocity(MIN_VEL, SMART_MOTION_SLOT);
    m_rightController.setSmartMotionMaxAccel(MAX_ACC, SMART_MOTION_SLOT);
    m_rightController.setSmartMotionAllowedClosedLoopError(ALLOWED_ERR, SMART_MOTION_SLOT);
  }
  
  public double getLeftPos() {
    return m_leftTeleEnc.getPosition();
  }

  public double getRightPos() {
    return m_rightTeleEnc.getPosition();
  }

  public void setLeftTele() {
    m_leftTele.set(-0.15);
  }

  public void setRightTele() {
    m_rightTele.set(-0.15);
  }

  public void extendPivot() {
    m_pivot.set(Value.kForward);
  }

  public void retractPivot() {
    m_pivot.set(Value.kReverse);
  }

  /**
   * Sets the telescopes to begin moving towards their highest extension, e.g. step 1 of the climbing sequence
   */
  public void teleHigh() {
    m_leftController.setReference(MAX_HEIGHT, ControlType.kSmartMotion);
    m_rightController.setReference(MAX_HEIGHT + 3.5, ControlType.kSmartMotion);
  }
  /**
   * Sets the telescopes to begin moving towards their lowest extension, e.g. just above fully stowed
   */
  public void teleLow() {
    m_leftController.setReference(MIN_HEIGHT, ControlType.kSmartMotion);
    m_rightController.setReference(MIN_HEIGHT + 2.5, ControlType.kSmartMotion);
  }
  /**
   * Sets the telescopes to begin moving towards just being clear of the bar to transfer
   * the hold to the pivot arms
   */
  public void teleStage() {
    m_leftController.setReference(STAGE_HEIGHT, ControlType.kSmartMotion);
    m_rightController.setReference(STAGE_HEIGHT + 1.5, ControlType.kSmartMotion);
  }

  public void stopLeft() {
    m_leftTele.stopMotor();
  }

  public void stopRight() {
    m_rightTele.stopMotor();
  }

  public void resetEncoders() {
    m_leftTeleEnc.setPosition(0);
    m_rightTeleEnc.setPosition(0);
  }

  /**
   * Get the state of the left telescope hall effect switch
   * @return Whether or not the hall effect is "pressed"
   */
  public boolean getLeftLimit() {
    return m_leftLimit.isPressed();
  }
  /**
   * Get the state of the right telescope hall effect switch
   * @return Whether or not the hall effect is "pressed"
   */
  public boolean getRightLimit() {
    return m_rightLimit.isPressed();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left tele pos", getLeftPos());
    SmartDashboard.putNumber("Right tele pos", getRightPos());
    SmartDashboard.putNumber("Left output", m_leftTele.getAppliedOutput());
    SmartDashboard.putNumber("Right output", m_rightTele.getAppliedOutput());
  }
}