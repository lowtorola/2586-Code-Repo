package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {
    Compressor comp = new Compressor();

    public PneumaticSubsystem() {
        comp.start();
    }
}