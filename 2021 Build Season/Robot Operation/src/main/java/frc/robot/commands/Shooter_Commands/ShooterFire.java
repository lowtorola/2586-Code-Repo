package frc.robot.commands.Shooter_Commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.HoodLogic;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterFire extends ParallelCommandGroup {
    
    private ShooterSubsystem m_shooterSub = new ShooterSubsystem();
    private LimelightSubsystem m_llSub = new LimelightSubsystem();

    public ShooterFire(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
        m_shooterSub = shooterSubsystem;
        m_llSub = limelightSubsystem;

        addCommands(
            new HoodLogic(m_shooterSub, m_llSub),
            new PIDCommand(
            new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD),
            // Close the loop on Shooter Current RPM
            m_shooterSub::getMeasurement,
            // get setpoint
            m_shooterSub.getTargetRPM(m_llSub.getDistance()),
            // pipe the output to the shooter motor
            output -> m_shooterSub.setShooterVolts((output + m_shooterSub.getFeedForward(m_shooterSub.getTargetRPM(m_llSub.getDistance()))))
        ));

    }


}
