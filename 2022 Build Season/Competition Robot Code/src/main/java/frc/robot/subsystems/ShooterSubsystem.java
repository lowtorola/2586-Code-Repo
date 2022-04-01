// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public class ShooterSubsystem extends SubsystemBase {

    private final Spark m_feeder = new Spark(FEEDER);
    private final CANSparkMax m_flywheel = new CANSparkMax(FLYWHEEL, MotorType.kBrushless);
    private final DigitalInput m_topBB = new DigitalInput(TOP_BB);
    private final DigitalInput m_bottomBB = new DigitalInput(BOTTOM_BB);
    private final DigitalInput m_indexBB = new DigitalInput(INDEX_BB);
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_pidController;
    int feederState;
    double m_targetRPM;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    m_encoder = m_flywheel.getEncoder();
    m_feeder.setInverted(true);
    m_flywheel.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40);
    m_flywheel.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 40);

    m_targetRPM = 0.0;

    m_pidController = m_flywheel.getPIDController();
    m_pidController.setP(KP);
    m_pidController.setI(KI);
    m_pidController.setD(KD);
    m_pidController.setIZone(KIZ);
    m_pidController.setFF(KFF);
    m_pidController.setOutputRange(KMIN_OUTPUT, KMAX_OUTPUT);
  }

  /**
   * Runs the shooter at a constant voltage
   */
  public void shootVolts() {
      m_flywheel.setVoltage(SHOOT_VOLTS);
  }

  /**
   * Sets the shooter to a given speed (RPM)
   */
  public void shootRPM(double speed) {
    m_targetRPM = speed;
    m_pidController.setReference(m_targetRPM, ControlType.kVelocity);
  }

  /**
   * Sets the shooter to a calculated RPM value
   */
  public void shootAuto(double vertAngleError) {
    m_targetRPM = quadReg(vertAngleError);
    m_pidController.setReference(m_targetRPM, ControlType.kVelocity);
  }

  /**
   * Run the feeder at shoot speed
   */
  public void feederFwd() {
      m_feeder.set(FEEDER_FWD);
  }

  /**
   * Run the feeder at indexing (slower) speed
   */
  public void feederIndex() {
      m_feeder.set(FEEDER_INDEX);
  }

  /**
   * Run the feeder at reverse speed
   */
  public void feederRev() {
      m_feeder.set(FEEDER_REV);
  }

  /**
   * stop only the shooter flywheel
   */
  public void stopFlywheel() {
      m_flywheel.stopMotor();
  }

  /**
   * stop the shooter feeder 
   */
  public void stopFeeder() {
      m_feeder.set(0);
  }

  /**
   * Returns the state of the bottom shooter beam break
   * @return Returns true when ball present, false when no ball
   */
  public boolean getBottomBB() {
    return !m_bottomBB.get();
  }

    /**
   * Returns the state of the top shooter beam break
   * @return Returns true when ball present, false when no ball
   */
  public boolean getTopBB() {
    return !m_topBB.get();
  }

    /**
   * Returns the state of the top shooter beam break
   * @return Returns true when ball present, false when no ball
   */
  public boolean getIndexBB() {
    return !m_indexBB.get();
  }

  public int getFeederState() {
    return feederState;
  }

  public double getVelocity() {
      return m_encoder.getVelocity();
  }

  public boolean atSpeed() {
      return Math.abs(m_targetRPM - getVelocity()) < TOLERANCE_RPM;// (getVelocity() >= (SHOOT_RPM - 500));
  }
/**
 * Quadratic regression for rpm
 * @param input angle error from LL
 * @return target RPM
 */
  public double quadReg(double input) {
    if(input > 5.0) {
      return 2620.0;
    } else {
    return ((KQUADRATIC * Math.pow(input, 2)) + (input * KLINEAR) + KCONSTANT);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(getIndexBB() && !getTopBB() && !getBottomBB()) {
        feederState = 1;         
    } else if(getIndexBB() && getBottomBB() && !getTopBB()) {
        feederState = 2;
    } else if(!getIndexBB() && getBottomBB() && getTopBB()) {
        feederState = 3;
    } else if(!getIndexBB() && getBottomBB() && !getTopBB()) {
        feederState = 4;
    } else if(!getIndexBB() && !getBottomBB() && getTopBB()) {
        feederState = 5;
    } else {
        feederState = 0;
    }
    // update at speed value periodically
    // atSpeed = Math.abs(m_targetRPM - getVelocity()) < TOLERANCE_RPM;
    SmartDashboard.putNumber("Shooter RPM", m_encoder.getVelocity());
    SmartDashboard.putNumber("Target RPM", m_targetRPM);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
