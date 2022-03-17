// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {

  private Spark m_intakeRoller = new Spark(ROLLER_MOTOR);
  private DoubleSolenoid m_intakeCylinder = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, CYLINDER[0], CYLINDER[1]);

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    m_intakeCylinder.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Run the intake roller to intake cargo at a constant speed.
   */
  public void intake() {
      m_intakeRoller.set(FWD_SPEED);
  }

  /**
   * Run the intake roller to reverse at a slower, constant speed.
   */
  public void reverse() {
      m_intakeRoller.set(REV_SPEED);
  }

  /**
   * Lower the intake.
   */
  public void extend() {
      m_intakeCylinder.set(Value.kForward);
  }

  /**
   * Raise the intake.
   */
  public void retract() {
      m_intakeCylinder.set(Value.kReverse);
  }

  /**
   * Stop the intake roller.
   */
  public void stop() {
      m_intakeRoller.set(0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
