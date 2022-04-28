package frc.robot.subsystems;

import frc.robot.lib.Limelight;
import frc.robot.lib.ControlMode.CamMode;
import frc.robot.lib.ControlMode.LedMode;

import static frc.robot.Constants.LimelightConstants.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase{
 
    private final Limelight m_limelight;

    private NetworkTable m_table;
    private String m_tableName;

    public LimelightSubsystem() {

        m_tableName = "limelight";
        m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
        
        m_limelight = new Limelight();
     //   m_limelight.setLEDMode(LedMode.kforceOff);
     //   m_limelight.setCamMode(CamMode.kdriver);
        limelightAimConfig(); // FIXME: start with LL on, keep it on!
        System.out.println("LL set!");
    }

  public double getAngleErrorX() {
      return m_limelight.getdegRotationToTarget();
  }

  public double getAngleErrorY() {
      return m_limelight.getdegVerticalToTarget();
  }

  /**
   * Get the percent of max error in the robot rotation. Capped to a max measurable angle (bc of distance target size changes)
   * @return Percent of max rotation error currently observed
   */
  public double getPercentErrorX() {
    if(Math.abs(getAngleErrorX()) > MAX_ANGLE_ERROR_X) {
        if(getAngleErrorX() < 0) {
            return -1.0;
        } else {
            return 1.0;
        }
    } else {
        return getAngleErrorX() / MAX_ANGLE_ERROR_X;
    }
  }

    /**
   * Get the percent of max error in the distance to target. Capped to a max measurable angle (bc of overshoot)
   * @return Percent of max dist to target error currently observed
   */
  public double getPercentErrorY() {
    if(Math.abs(getAngleErrorY()) > MAX_ANGLE_ERROR_Y) {
        if(getAngleErrorY() < 0) {
            return -1.0;
        } else {
            return 1.0;
        }
    } else {
        return getAngleErrorY() / MAX_ANGLE_ERROR_Y;
    }
  }

  public boolean inPositionX() {
      return Math.abs(getAngleErrorX()) <= TOLERANCE_ERROR_X;
  }

  public boolean inPositionY() {
      return Math.abs(getAngleErrorY()) <= TOLERANCE_ERROR_Y;
  }

  public boolean hasTarget() {
      return m_limelight.getIsTargetFound();
  }

  public void limelightDriveConfig() {
        m_limelight.setPipeline(DISABLED_PIPELINE);
        m_limelight.setLEDMode(LedMode.kforceOff);
      //  m_limelight.setCamMode(CamMode.kdriver);
  }

  public void limelightAimConfig() {
        m_limelight.setPipeline(AIM_PIPELINE);
        m_limelight.setLEDMode(LedMode.kforceOn);
  }

  @Override
  public void periodic() {
      SmartDashboard.putBoolean("In Position", inPositionX());
  }

}
