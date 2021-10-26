// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveDrive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private WheelDrive backRight = new WheelDrive(7, 8, 12);
  private WheelDrive backLeft = new WheelDrive(3, 4, 10);
  private WheelDrive frontRight = new WheelDrive(5, 6, 11);
  private WheelDrive frontLeft = new WheelDrive(1, 2, 9);

  private SwerveDrive swerveDrive = new SwerveDrive(backRight, backLeft, frontRight, frontLeft);

  private Joystick joystick = new Joystick(0);

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("FL", frontLeft.getEncValue());
    SmartDashboard.putNumber("FR", frontRight.getEncValue());
    SmartDashboard.putNumber("RL", backLeft.getEncValue());
    SmartDashboard.putNumber("RR", backRight.getEncValue());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    swerveDrive.drive(joystick.getRawAxis(0), joystick.getRawAxis(1), joystick.getRawAxis(2));

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
