package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.limelight;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShoot extends ParallelCommandGroup {
    
    public AutoShoot(ShooterSubsystem shooterSub, DriveSubsystem driveSub, LimelightSubsystem limelight, IndexerSubsystem indexerSub, IntakeSubsystem intakeSub) {

        addCommands(
            new ShooterSpool(shooterSub, limelight),
            new HoodLogic(limelight, shooterSub),
            new LimelightTarget(LimelightConstants.kTargetAngle, driveSub, limelight),
            new SequentialCommandGroup(
                new WaitForShooter(shooterSub, () -> limelight.getErrorY()),
                new ParallelCommandGroup(
                    new InstantCommand(indexerSub::runIndexer, indexerSub),
                    new InstantCommand(shooterSub::runFeed))),
          new SequentialCommandGroup(
            new WaitForExit(shooterSub),
            new InstantCommand(intakeSub::startIntake, intakeSub))
        );
    }
}
