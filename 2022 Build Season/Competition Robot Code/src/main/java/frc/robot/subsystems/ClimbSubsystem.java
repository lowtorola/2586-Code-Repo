// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsystem extends SubsystemBase {

  private final CANSparkMax m_teleRight = new CANSparkMax(RIGHT_TELESCOPE, MotorType.kBrushless);
  private final CANSparkMax m_teleLeft = new CANSparkMax(LEFT_TELESCOPE, MotorType.kBrushless);
  private final RelativeEncoder m_leftTeleEnc;
  private final RelativeEncoder m_rightTeleEnc;

  private final DoubleSolenoid m_leftPivot = new DoubleSolenoid(PneumaticsModuleType.REVPH,PIVOT_LEFT[0],PIVOT_LEFT[1]);
  private final DoubleSolenoid m_rightPivot = new DoubleSolenoid(PneumaticsModuleType.REVPH,PIVOT_RIGHT[0],PIVOT_RIGHT[1]);

  private final ProfiledPIDController m_leftController;
  private final ProfiledPIDController m_rightController;


  public ClimbSubsystem() {

    m_leftTeleEnc = m_teleLeft.getEncoder();
    m_rightTeleEnc = m_teleRight.getEncoder();
    m_leftTeleEnc.setPositionConversionFactor(WINCH_DRUM);
    m_rightTeleEnc.setPositionConversionFactor(WINCH_DRUM);

    m_teleLeft.setSoftLimit(SoftLimitDirection.kForward, MAX_HEIGHT);
    m_teleRight.setSoftLimit(SoftLimitDirection.kForward, MAX_HEIGHT);

    m_leftController = new ProfiledPIDController(L_GAINS[0], L_GAINS[1], L_GAINS[2], L_CONSTRAINTS); // INCHES!!!
    m_rightController = new ProfiledPIDController(R_GAINS[0], R_GAINS[1], R_GAINS[2], R_CONSTRAINTS); // INCHES!!
    m_leftController.setTolerance(0.5); // inches!!
    m_rightController.setTolerance(0.5); // inches!!

  }
  
  public double getLeftPos() {
    return m_leftTeleEnc.getPosition();
  }

  public double getRightPos() {
    return m_rightTeleEnc.getPosition();
  }

  public void setLeftTele(double output) {
    m_teleLeft.set(output);
  }

  public void setRightTele(double output) {
    m_teleRight.set(output);
  }

  public void extendPivot() {
    m_leftPivot.set(Value.kForward);
    m_rightPivot.set(Value.kForward);
  }

  public void retractPivot() {
    m_leftPivot.set(Value.kReverse);
    m_rightPivot.set(Value.kReverse);
  }

  /**
   * Sets the telescopes to begin moving towards their highest extension, e.g. step 1 of the climbing sequence
   */
  public void teleHigh() {
    m_leftController.setGoal(MAX_HEIGHT);
    m_rightController.setGoal(MAX_HEIGHT);
    setLeftTele(m_leftController.calculate(getLeftPos()));
    setRightTele(m_rightController.calculate(getRightPos()));
  }
  /**
   * Sets the telescopes to begin moving towards their lowest extension, e.g. just above fully stowed
   */
  public void teleLow() {
    m_leftController.setGoal(MIN_HEIGHT);
    m_rightController.setGoal(MIN_HEIGHT);
    setLeftTele(m_leftController.calculate(getLeftPos()));
    setRightTele(m_rightController.calculate(getRightPos()));
  }
  /**
   * Sets the telescopes to begin moving towards just being clear of the bar to transfer
   * the hold to the pivot arms
   */
  public void teleStage() {
    m_leftController.setGoal(STAGE_HEIGHT);
    m_rightController.setGoal(STAGE_HEIGHT);
    setLeftTele(m_leftController.calculate(getLeftPos()));
    setRightTele(m_rightController.calculate(getRightPos()));
  }

  public void teleStop() {
    m_teleLeft.set(0);
    m_teleRight.set(0);
  }

  /**
   * Allows access to the left telescope's ProfiledPIDController
   * @return The ProfiledPIDController object responsible for controlling the telescope
   */
  public ProfiledPIDController getLeftController() {
    return m_leftController;
  }
  /**
   * Allows access to the right telescope's ProfiledPIDController
   * @return The ProfiledPIDController object responsible for controlling the telescope
   */
  public ProfiledPIDController getRightController() {
    return m_rightController;
  }

}