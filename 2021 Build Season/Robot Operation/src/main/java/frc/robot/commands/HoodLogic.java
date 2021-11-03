package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.limelight;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class HoodLogic extends CommandBase{
    private double llAngleError;
    private ShooterSubsystem m_shooter;
    private LimelightSubsystem m_limelight;

    public HoodLogic(LimelightSubsystem limelight, ShooterSubsystem shooterSub) {
        m_limelight = limelight;
        m_shooter = shooterSub;
    }

    @Override
    public void execute() {
        llAngleError = m_limelight.getErrorY();

        m_shooter.hoodLogic(llAngleError);
    }

}
