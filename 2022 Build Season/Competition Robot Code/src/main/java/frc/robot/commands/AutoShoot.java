package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShoot extends ParallelCommandGroup {

    private Runnable m_shooterCommand;
    private ShooterSubsystem m_shooter;

    public AutoShoot(Runnable shooterCommand, ShooterSubsystem shooter) {
        this.m_shooterCommand = shooterCommand;
        this.m_shooter = shooter;

        addCommands(
            new InstantCommand(m_shooterCommand), 
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new InstantCommand(m_shooter::feederFwd),
                new WaitCommand(2)
                
            ));
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopFlywheel();
        m_shooter.stopFeeder();
    }
}
