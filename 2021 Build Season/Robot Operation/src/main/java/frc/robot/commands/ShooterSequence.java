package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.HoodLogic;
import frc.robot.commands.Shooter_Commands.ShooterFire;

public class ShooterSequence extends ParallelCommandGroup {


    public ShooterSequence(ShooterSubsystem m_shooterSub, IndexerSubsystem m_indexerSub, IntakeSubsystem m_intakeSub, LimelightSubsystem m_llSub) {

        addCommands(
          
          new ShooterFire(m_shooterSub, m_llSub),
          new SequentialCommandGroup(
            new WaitForShooter(m_shooterSub),
            new ParallelCommandGroup(
              new InstantCommand(m_indexerSub::runIndexer, m_indexerSub),
              new InstantCommand(m_shooterSub::runFeeder))),
          new SequentialCommandGroup(
            new WaitForExit(m_shooterSub),
            new InstantCommand(m_intakeSub::startIntake, m_intakeSub)));

    }
}
