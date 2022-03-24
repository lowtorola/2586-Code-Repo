// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AdvanceFeeder extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooter;
  int feederState = 0;
  int startingState;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AdvanceFeeder(ShooterSubsystem shooter) {
    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startingState = m_shooter.getFeederState();

    feederState = startingState;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    feederState = m_shooter.getFeederState();
    SmartDashboard.putNumber("Feeder State", feederState);

    // index if in state 1 or 2, otherwise stop
    switch(feederState) {
      case 1:
      m_shooter.feederIndex();
      break;
      case 2:
      m_shooter.feederIndex();
      break;
      default:
      m_shooter.stopFeeder();
      break;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopFeeder();
  }
}
