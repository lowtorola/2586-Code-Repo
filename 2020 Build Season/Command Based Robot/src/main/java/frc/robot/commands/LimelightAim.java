package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.LimelightConstants;

public class LimelightAim extends PIDCommand {
    
    public LimelightAim(double targetAngleDegrees, DriveSubsystem drive) {
        super(
            new PIDController(LimelightConstants.kTurnP, LimelightConstants.kTurnI, LimelightConstants.kTurnD),
            // Close loop on heading
            drive::getLimelightAngleDegX,
            // Set reference to target
            targetAngleDegrees,
            // Pipe output to turn robot
            output -> drive.arcadeDrive(0, output),
            // Require the drive
            drive);
    
        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController()
            .setTolerance(LimelightConstants.kTurnToleranceDeg);
      }
}