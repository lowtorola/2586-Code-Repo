package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.lib.limelight;

public class LimelightSubsystem extends SubsystemBase {

    private final limelight limelight = new limelight();

    public LimelightSubsystem() {
        limelight.setPipeline(0);
        limelight.setLEDMode(0);
        System.out.println("LL set");
    }

    public double getErrorX() {
        return limelight.getTX();
    }

    public double getErrorY() {
        return limelight.getTY();
    }

    public double getDistance() {
        return  (
        LimelightConstants.kInputSquared*(limelight.getTY())) 
        + (LimelightConstants.kInputScaled*(limelight.getTY()))
        + LimelightConstants.kConstant; // LimelightConstants.kCalcHeight / (Math.tan(LimelightConstants.kBaseDegree + limelight.getTY()))
    }

    public void setDisabledPipeline() {
        limelight.setPipeline(LimelightConstants.kDisabledPipeline);
    }

    public void setCompTarget() {
        limelight.setPipeline(LimelightConstants.kCompTarget);
    }

    public void setPractice() {
        limelight.setPipeline(2);
        System.out.println("Practice Pipeline set");
    }
    
}
