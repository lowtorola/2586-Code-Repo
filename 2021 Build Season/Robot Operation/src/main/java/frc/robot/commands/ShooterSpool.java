package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterSpool extends PIDCommand {
    ShooterSubsystem m_shooter;
    LimelightSubsystem m_llSub;
    public ShooterSpool(ShooterSubsystem shooterSub, LimelightSubsystem llSub) {
    super(
        new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD),
        // Close the loop on Shooter Current RPM
        shooterSub::getMeasurement,
        // get setpoint
        () -> shooterSub.getTargetRPM(llSub.getErrorY()),
        // pipe the output to the shooter motor
        output -> shooterSub.runShooterRPM(output + shooterSub.getFeedForward(ShooterConstants.kTargetRPM))
    );
    m_shooter = shooterSub;
    m_llSub = llSub;
} 

@Override
public void execute() {
  System.out.println(m_controller.getSetpoint());
  m_useOutput.accept(
      m_controller.calculate(m_measurement.getAsDouble(), m_setpoint.getAsDouble()));
      System.out.println();
}

}
