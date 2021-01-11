/**
 * This software was developed by Gabriel McMillan, and depends on software from WPILib and NTCore, so keep that in mind when using.
 * The GPL 3.0 license applies to this program, you are free to distribute, use, and modify this software however you wish, but keep this notice,
 * and the license file with it when distributing it. Also, feel free to make any pull requests to enhance this library!
 * Good luck, and have a great season! ~Gabriel McMillan
 */

package frc.robot.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limelight {
  private String limelightName;
  NetworkTableInstance getNT = NetworkTableInstance.getDefault();
  private NetworkTable limelightNT;

  public limelight() {
    limelightName = "limelight";
    limelightNT = getNT.getTable(limelightName);
  }

  public limelight(String name) {
    limelightName = name;
    limelightNT = getNT.getTable(limelightName);
  }

  /**
   * setLEDMode() - Sets LED mode. 0 use the LED Mode set in the current pipeline,
   * 1 force off, 2 force blink, 3 force on
   * 
   * @return void
   */
  public void setLEDMode(double value) {
    limelightNT.getEntry("ledMode").setNumber(value);
  }

  /**
   * setCAMMode() - Sets camera mode. 0 for Vision processor, 1 for Driver Camera
   * (Increases exposure, disables vision processing)
   * 
   * @return void
   */
  public void setCAMMode(double value) {
    limelightNT.getEntry("camMode").setNumber(value);
  }

  /**
   * setPipeline() Sets current pipeline on Limelight.
   * 
   * @param Pipeline number
   */
  public void setPipeline(double value) {
    limelightNT.getEntry("pipeline").setNumber(value);
  }

  /**
   * setStreamMode() - Sets limelight’s streaming mode -0 Standard - Side-by-side
   * streams if a webcam is attached to Limelight - 1 PiP Main - The secondary
   * camera stream is placed in the lower-right corner of the primary camera
   * stream - 2 PiP Secondary - The primary camera stream is placed in the
   * lower-right corner of the secondary camera stream
   * 
   * @return void
   */
  public void setStreamMode(double value) {
    limelightNT.getEntry("stream").setNumber(value);
  }

  /**
   * getTV() - Monitor whether the limelight has any valid targets (0 or 1)
   * 
   * @return 1 if limelight target lock, 0 if no lock.
   */
  public double getTV() {
    return limelightNT.getEntry("tv").getDouble(0);
  }

  /**
   * getTA() - Target Area (0% of image to 100% of image)
   * 
   * @return (0-100)
   */
  public double getTA() {
    return limelightNT.getEntry("ta").getDouble(0);
  }

  /**
   * getTX() - monitor the Limelight's TX Value
   * 
   * @return Horizontal Offset From Crosshair To Target (-29.8 to 29.8 degrees)
   */
  public double getTX() {
    return limelightNT.getEntry("tx").getDouble(0);
  }

  /**
   * getTY() - monitor the Limelight's TY Value
   * 
   * @return Vertical Offset From Crosshair To Target (-24.85 to 24.85 degrees)
   */
  public double getTY() {
    return limelightNT.getEntry("ty").getDouble(0);
  }

  /**
   * getPipeline() - monitor the Limelight's pipeline Value
   * 
   * @return pipeline
   */
  public double getPipeline() {
    return limelightNT.getEntry("pipeline").getDouble(0);
  }

  /**
   * getCameraTranslation() - Results of a 3D position solution
   * 
   * @return 6 numbers: Translation (x,y,y) Rotation(pitch,yaw,roll)
   */
  public double getCameraTranslation() {
    return limelightNT.getEntry("camtran").getDouble(0);
  }

  /**
   * get() - monitor any value needed outside of currently provided.
   * 
   * @param key to pull
   * @return value of key
   */
  public double get(String entry) {
    return limelightNT.getEntry(entry).getDouble(0);
  }

  /**
   * setSnapshot - Allows users to take snapshots during a match
   * 
   * @param 0 for no, 1 for 2 snapshots per second.
   * @return False if the table key already exists with a different type
   */
  public boolean setSnapshot(double value) {
    return limelightNT.getEntry("snapshot").setValue(value);
  }

  /**
   * set() - Set any value outside what is currently provided with the Limelight
   * 
   * @return False if the table key already exists with a different type
   * @param key to set, and value to set.
   */
  public boolean set(String entry, Double value) {
    return limelightNT.getEntry(entry).setNumber(value);
  }

  /**
   * getDist() - calculates approximate distance from a fixed angled limelight to
   * the target.
   * 
   * @param targetHeight= target height in meters, limelightHeight = height of limelight from the ground in meters, limelightAngle =
   *           angle in degrees of the limelight on the robot.
   * @return approx distance in meters
   */
  public double getDist(double targetHeight, double limelightHeight, double limelightAngle) {
    double a2 = getTY();
    double currentDist = (Math.abs(targetHeight - limelightHeight) / Math.tan(limelightAngle + a2));
    return currentDist;
  }
}