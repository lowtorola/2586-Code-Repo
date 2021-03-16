package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPreload extends CommandBase {
    private final ShooterSubsystem shooterControl;

    public ShooterPreload(ShooterSubsystem subsystem) {
        shooterControl = subsystem;
    }

    @Override
    public void execute() {
        if (shooterControl.getFeederBB()) {
            shooterControl.preloadShooter();
        } else {
            shooterControl.stopFeed();
        }
    }

    @Override
    public boolean isFinished() {
        if (!shooterControl.getFeederBB()) {
            return true;
        } else {
            return false;
        }
    }

}