package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraight extends CommandBase {

    private final DriveSubsystem driveBase;

    private final PIDController drive = new PIDController(
        DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD);
    private final PIDController turn = new PIDController(
        DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);

    public DriveStraight(DriveSubsystem driveSub, double driveDist) {
        driveBase = driveSub;
        drive.setSetpoint(driveDist);
        turn.setSetpoint(0);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(drive.getPositionError()) < 1) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void execute() {
        driveBase.arcadeDrive(drive.calculate(driveBase.getDistAvg()), turn.calculate(driveBase.getAngle()));
    }
}