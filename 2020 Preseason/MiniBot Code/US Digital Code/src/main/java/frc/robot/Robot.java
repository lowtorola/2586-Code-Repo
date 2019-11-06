/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  Encoder leftEnc;
  
  Encoder rightEnc;

  Joystick joy1;

  

  @Override
  public void robotInit() {

    final double kCountsPerRevolution = 800;
    final double kInchesPerRevolution = 12.56;
    final double kInchesPerCount = kInchesPerRevolution / kCountsPerRevolution;

    leftEnc = new Encoder(6, 7);
    leftEnc.setDistancePerPulse(kInchesPerCount);
    leftEnc.setSamplesToAverage(3);
    leftEnc.reset();

    rightEnc = new Encoder(8, 9);
    rightEnc.setDistancePerPulse(kInchesPerCount);
    rightEnc.setReverseDirection(true);
    rightEnc.setSamplesToAverage(3);
    rightEnc.reset();

    joy1 = new Joystick(0);

  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    sensorPrints();

    if (joy1.getRawButton(1)) {
      leftEnc.reset();
      rightEnc.reset();
    }

  }

  @Override
  public void testInit() {
  }

  public void sensorPrints() {
    
    SmartDashboard.putNumber("Left Distance", leftEnc.getDistance());
    SmartDashboard.putNumber("Right Distance", rightEnc.getDistance());
    
  }

  @Override
  public void testPeriodic() {
  }

}
