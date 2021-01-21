package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class WaitForShooter extends CommandBase {
    private final ShooterSubsystem shooter;
    public WaitForShooter(ShooterSubsystem shooterControl2) {
        shooter = shooterControl2;
    }
    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("Shooter At Speed", shooter.isAtSpeed());
        return shooter.isAtSpeed();
    }
}