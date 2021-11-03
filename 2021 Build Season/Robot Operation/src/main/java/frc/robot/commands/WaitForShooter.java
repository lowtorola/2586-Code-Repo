package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class WaitForShooter extends CommandBase {
    private final ShooterSubsystem shooter;
    private DoubleSupplier currentError;
    public WaitForShooter(ShooterSubsystem shooterControl2, DoubleSupplier angleError) {
        shooter = shooterControl2;
        currentError = angleError;
    }
    @Override
    public boolean isFinished() {
        System.out.println(currentError.getAsDouble());
        return shooter.isAtSpeed(shooter.getTargetRPM(currentError.getAsDouble()));
    }
}