package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightTarget extends CommandBase {

    private final LimelightSubsystem m_limelight;
    private final Drivetrain m_drivetrain;
    private double m_angleErrorX;
    private double m_angleErrorY;

    private ChassisSpeeds m_targetSpeeds;

    public LimelightTarget(LimelightSubsystem limelight, Drivetrain drivetrain) {
        this.m_limelight = limelight;
        this.m_drivetrain = drivetrain;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_limelight.limelightAimConfig();
      //  m_angleErrorX = m_limelight.getAngleErrorX();
      //  m_angleErrorY = m_limelight.getAngleErrorY();
        m_targetSpeeds = new ChassisSpeeds();
    }

    @Override
    public void execute() {
        m_limelight.limelightAimConfig();
        m_angleErrorX = m_limelight.getAngleErrorX();
        m_angleErrorY = m_limelight.getAngleErrorY();
        
        m_targetSpeeds = new ChassisSpeeds(
            m_limelight.getPercentErrorY() * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND * 0.25, // change scale factor
            0.0,
            m_limelight.getPercentErrorX() * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * -0.2 // change scale factor
        );
        m_drivetrain.driveFromSpeeds(m_targetSpeeds, false);
    }

    @Override
    public boolean isFinished() {
        if(m_limelight.inPositionX() && m_limelight.inPositionY() && m_limelight.hasTarget()) {
            m_limelight.limelightDriveConfig();
            return true;
        } else {
            return false;
        }
    }

}
