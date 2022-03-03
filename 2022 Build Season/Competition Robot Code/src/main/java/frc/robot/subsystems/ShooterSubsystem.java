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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

    private final Spark m_feeder = new Spark(FEEDER);
    private final CANSparkMax m_flywheel = new CANSparkMax(FLYWHEEL, MotorType.kBrushless);
    private final DigitalInput m_topBB = new DigitalInput(TOP_BB);
    private final DigitalInput m_bottomBB = new DigitalInput(BOTTOM_BB);
    private RelativeEncoder m_encoder;

    int feederState;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    m_encoder = m_flywheel.getEncoder();
  }

  /**
   * Runs the shooter at a constant voltage
   */
  public void shootVolts() {
      m_flywheel.setVoltage(SHOOT_VOLTS);
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
      m_flywheel.set(0);
  }

  /**
   * stop the shooter feeder
   * 
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

  public double getVelocity() {
    return m_encoder.getVelocity();
}

  public boolean atSpeed() {
    return (getVelocity() >= (SHOOT_RPM - 50));
}

    /**
   * Returns the state of the top shooter beam break
   * @return Returns true when ball present, false when no ball
   */
  public boolean getTopBB() {
    return !m_topBB.get();
  }

  public int getFeederState() {
    return feederState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(getBottomBB() && !getTopBB()) {
        feederState = 1;         
    } else if(getBottomBB() && getTopBB()) {
        feederState = 2;
    } else if(!getBottomBB() && getTopBB()) {
        feederState = 3;
    } else {
        feederState = 0;
    }

    

    SmartDashboard.putNumber("feeder state", feederState);
    SmartDashboard.putBoolean("top bb", getTopBB());
    SmartDashboard.putBoolean("bottom bb", getBottomBB());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
