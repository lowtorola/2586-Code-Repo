package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;

public class LimelightTarget extends CommandBase {
    private final DriveSubsystem driveBase;
    
    private final PIDController drive = new PIDController(
        LimelightConstants.kDriveP, LimelightConstants.kDriveI, LimelightConstants.kDriveD);
    private final PIDController turn = new PIDController(
        LimelightConstants.kTurnP, LimelightConstants.kTurnI, LimelightConstants.kTurnD);

        public LimelightTarget(DriveSubsystem driveSub) {
            driveBase = driveSub;
            drive.setSetpoint(0);
            turn.setSetpoint(0);
            drive.setTolerance(LimelightConstants.kDriveToleranceDeg);
            turn.setTolerance(LimelightConstants.kTurnToleranceDeg);
        }

    @Override
    public void execute() {
        driveBase.limelightAimConfig(LimelightConstants.kPracticePipeline);
        driveBase.arcadeDrive(
            drive.calculate(driveBase.getLimelightAngleDegY()), 
            drive.calculate(driveBase.getLimelightAngleDegX()));
    }

    @Override
    public boolean isFinished() {
        if(drive.getPositionError() < LimelightConstants.kDriveToleranceDeg 
        && turn.getPositionError() < LimelightConstants.kTurnToleranceDeg) {
            return true;
        } else {
            return false;
        }
    }

}