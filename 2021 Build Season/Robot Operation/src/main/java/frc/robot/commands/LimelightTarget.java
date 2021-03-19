package frc.robot.commands;

import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LimelightConstants;
import frc.robot.lib.limelight;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LimelightTarget extends PIDCommand {

    private LimelightSubsystem limelight;
    private DriveSubsystem m_drive;

        public LimelightTarget(double targetAngleDegrees, DriveSubsystem driveSub, LimelightSubsystem limelightSub) {
            super(
            new PIDController(LimelightConstants.kP, LimelightConstants.kI, LimelightConstants.kD),

            // close loop on heading
            limelightSub::getErrorX,
            // Target Angle
            targetAngleDegrees,
            // pipe output to drivebase
            output -> driveSub.arcadeDrive(0, -output),
            // require drivebase
            driveSub,
            // require limelight
            limelightSub
            );

            getController().enableContinuousInput(-180, 180);
            getController().setTolerance(LimelightConstants.kTurnToleranceDeg);

            driveSub.setMotorsCoast();

            limelight = limelightSub;
            m_drive = driveSub;
        }
 // TODO: try just setting LEDMode and not Pipeline!!!!
    @Override
    public void end(boolean interrupted) {
        System.out.println("limelight set to disabled");
        limelight.setDisabledPipeline();
    }
/*
    @Override
    public boolean isFinished() {
        // End when the controller is at the reference
        if (getController().getPositionError() == 0.0) {
            return false;
        } else {
    m_drive.setMotorsBrake();
    return getController().atSetpoint();
        }
    }
*/
    @Override
    public void initialize() {
        limelight.setPractice();
    }
}