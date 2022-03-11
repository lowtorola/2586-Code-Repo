package frc.robot.subsystems;

import frc.robot.lib.Limelight;
import static frc.robot.Constants.LimelightConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase{
 
    private final Limelight m_limelight;

    public LimelightSubsystem() {
        m_limelight = new Limelight();
        m_limelight.setLEDMode(0);
        m_limelight.setCAMMode(1);
        m_limelight.setPipeline(0);
        System.out.println("LL set!");
    }

    public double getDistFromTarget() {
        return m_limelight.getDist(TARGET_HEIGHT, LIMELIGHT_HEIGHT, LIMELIGHT_ANGLE);
  }

  public double getAngleErrorX() {
      return m_limelight.getTX();
  }

  public double getAngleErrorY() {
      return m_limelight.getTY();
  }

  public double getPercentErrorX() {
    return Math.abs(getAngleErrorX()) / MAX_ANGLE_ERROR_X;
  }

  public double getPercentErrorY() {
    return Math.abs(getAngleErrorY()) / MAX_ANGLE_ERROR_Y;
  }

  public boolean inPositionX() {
      return Math.abs(getAngleErrorX()) <= MAX_ANGLE_ERROR_X;
  }

  public boolean inPositionY() {
      return Math.abs(getAngleErrorY()) <= MAX_ANGLE_ERROR_Y;
  }

  public void limelightDriveConfig() {
        m_limelight.setPipeline(DISABLED_PIPELINE);
        m_limelight.setLEDMode(1);
        m_limelight.setCAMMode(1);
  }

  public void limelightAimConfig() {
        m_limelight.setPipeline(AIM_PIPELINE);
        // m_limelight.setLEDMode(3);
  }



}
