package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class FeederPreload extends CommandBase {
    private final ShooterSubsystem shooterControl;

    public FeederPreload(ShooterSubsystem subsystem) {
        shooterControl = subsystem;

    }

    @Override
    public void execute() {
        if (shooterControl.getBB() == true) {
            shooterControl.preloadFeeder();
        } else {
            shooterControl.stopFeeder();
        }
    }
}