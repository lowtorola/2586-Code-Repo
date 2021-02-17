package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

public class DefaultDrive extends CommandBase {
    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_fwd;
    private final DoubleSupplier m_rot;

    public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier fwd, DoubleSupplier rot) {
        m_drive = subsystem;
        m_fwd = fwd;
        m_rot = rot;
        addRequirements(m_drive);
      }
    
      @Override
      public void execute() {
        m_drive.arcadeDrive(m_fwd.getAsDouble(), m_rot.getAsDouble());
      }

}
