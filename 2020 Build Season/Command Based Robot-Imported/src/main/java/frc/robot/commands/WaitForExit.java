package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class WaitForExit extends CommandBase{
    private final ShooterSubsystem shooter;
    private boolean prevExitBB;
    private int ballExitCounter;
    public WaitForExit(ShooterSubsystem shooterSub) {
        shooter = shooterSub;
        prevExitBB = false;
        ballExitCounter = 0;
    }


    @Override
    public boolean isFinished() {
        if (shooter.getShooterExitBB() && !prevExitBB) {
            ballExitCounter++;
        } 
        if (ballExitCounter >= ShooterConstants.kBallsToExit) {
            ballExitCounter = 0;
            return true;
        } else {
            prevExitBB = shooter.getShooterExitBB();
            return false;
        }
    }
    @Override
    public void execute() {
        SmartDashboard.putNumber("Shooter Exit Count", ballExitCounter);
    }
}