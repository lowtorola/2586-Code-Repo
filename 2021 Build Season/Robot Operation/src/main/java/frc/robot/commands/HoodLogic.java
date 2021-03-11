package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class HoodLogic extends CommandBase {

    private ShooterSubsystem m_shooterSub = new ShooterSubsystem();
    private LimelightSubsystem m_llSub = new LimelightSubsystem();

    public HoodLogic(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
        m_shooterSub = shooterSubsystem;
        m_llSub = limelightSubsystem;


    }

    @Override
    public void execute() {

        if(m_llSub.getDistance() <= ShooterConstants.kDistanceSwitcher) {
            m_shooterSub.extendHood();
            } else {
            m_shooterSub.retractHood();
            }
        }

    @Override
    public boolean isFinished() {
        return true;
    }    
}
