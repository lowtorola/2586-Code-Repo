package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final Drivetrain m_drivetrain;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final boolean m_fieldOriented;

    public DriveCommand(Drivetrain drivetrain,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               boolean fieldOriented) {
        this.m_drivetrain = drivetrain;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_fieldOriented = fieldOriented;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrain.drive(
            -m_translationXSupplier.getAsDouble(),
            -m_translationYSupplier.getAsDouble(),
            -m_rotationSupplier.getAsDouble(),
            m_fieldOriented
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0.0, 0.0, 0.0, false);
    }
}
