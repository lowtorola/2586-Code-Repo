package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TestAuto extends ParallelCommandGroup {
    
    private Drivetrain m_drivetrain;
    private IntakeSubsystem m_intake;
    private ShooterSubsystem m_shooter;

    PathPlannerTrajectory testTrajectory = PathPlanner.loadPath("Multi Step Auto", 5.0, 3.0);
    
    public TestAuto(Drivetrain drivetrain, IntakeSubsystem intake, ShooterSubsystem shooter) {
        this.m_drivetrain = drivetrain;
        this.m_intake = intake;
        this.m_shooter = shooter;

        addCommands(
                new SequentialCommandGroup(
                    new InstantCommand(m_intake::extend),
                    new WaitCommand(1.2),
                    new InstantCommand(m_intake::intake),
                    new WaitCommand(1),
                    new InstantCommand(m_intake::stop),
                    new SequentialCommandGroup(
                        new WaitCommand(2),
                     //   new InstantCommand(m_shooter::shootVolts),
                        new WaitCommand(0.75),
                        new InstantCommand(m_shooter::feederFwd)
                    )).withTimeout(6),

                new PPSwerveControllerCommand(
                    testTrajectory, 
                    () -> m_drivetrain.m_odometry.getPoseMeters(), 
                    Drivetrain.m_kinematics, 
                    new PIDController(0.3, 0, .01), 
                    new PIDController(0.3, 0, .01), 
                    new ProfiledPIDController(0.4, 0, .01, new Constraints(Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 0.4)), 
                    (states) -> m_drivetrain.driveFromSpeeds(Drivetrain.m_kinematics.toChassisSpeeds(states)), 
                    m_drivetrain)
            );
    }
}
