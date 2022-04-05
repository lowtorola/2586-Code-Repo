package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class FollowPath extends ParallelCommandGroup {

    private Drivetrain m_drivetrain;
    private PathPlannerTrajectory m_trajectory;
    private PathPlannerState m_initialState;
    private Pose2d m_startingPose;

    public FollowPath(Drivetrain drivetrain, PathPlannerTrajectory trajectory) {
        // make member variables assigned to parameters
        this.m_drivetrain = drivetrain;
        this.m_trajectory = trajectory;

        // get initial calibration values
        this.m_initialState = m_trajectory.getInitialState();
        this.m_startingPose = new Pose2d(m_trajectory.getInitialPose().getTranslation(), m_initialState.holonomicRotation);

        addCommands(
            new PPSwerveControllerCommand(
                m_trajectory, 
                () -> m_drivetrain.getPose(), 
                Drivetrain.m_kinematics, 
                new PIDController(0.1, 0, 0.0), // tune this
                new PIDController(0.1, 0, 0.0), // and this
                new ProfiledPIDController(1.5, 0, 0.0, new Constraints(6.5, 4.5)), // and this....
                (states) -> m_drivetrain.driveFromSpeeds(Drivetrain.m_kinematics.toChassisSpeeds(states), false), 
                m_drivetrain)
            // home gyro before starting
            // .beforeStarting(new InstantCommand(() -> m_drivetrain.resetOdometry(m_startingPose)))
        );      
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.driveFromSpeeds(new ChassisSpeeds(), false);
    }
    
}