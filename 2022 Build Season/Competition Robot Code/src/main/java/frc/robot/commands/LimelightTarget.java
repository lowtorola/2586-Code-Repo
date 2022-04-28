package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightTarget extends CommandBase {

    private final LimelightSubsystem m_limelight;
    private final Drivetrain m_drivetrain;

    private ChassisSpeeds m_targetSpeeds;

    public LimelightTarget(LimelightSubsystem limelight, Drivetrain drivetrain) {
        this.m_limelight = limelight;
        this.m_drivetrain = drivetrain;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_limelight.limelightAimConfig();
        m_targetSpeeds = new ChassisSpeeds();
    }

    @Override
    public void execute() {
        m_limelight.limelightAimConfig();

        // check if we're inside of our shooting range. If we are, don't target range
        if(m_limelight.inPositionY()) {
            m_targetSpeeds = new ChassisSpeeds(
                0.0,
                0.0,
                -m_limelight.getPercentErrorX() * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.4
            );
            m_drivetrain.driveFromSpeeds(m_targetSpeeds, false);
        } else {
        m_targetSpeeds = new ChassisSpeeds(
            -m_limelight.getPercentErrorY() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * 0.35, // change scale factor
            0.0,
            -m_limelight.getPercentErrorX() * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.4 // change scale factor
        );
        m_drivetrain.driveFromSpeeds(m_targetSpeeds, false);
        }
    }

    @Override
    public boolean isFinished() {
        if(m_limelight.inPositionX() && m_limelight.inPositionY() && m_limelight.hasTarget()) {
          //  m_limelight.limelightDriveConfig();
            System.out.println("Locked On!!");
            return true;
        } else {
            return false;
        }
    }
    @Override
    public void end(boolean interrupted) {
        // we always want to go back to driver LL mode when we stop targeting
      //  m_limelight.limelightDriveConfig();
      m_drivetrain.driveFromSpeeds(new ChassisSpeeds(), false);
    }

}
