// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

public class Robot extends TimedRobot {
  private final Joystick m_controller = new Joystick(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumberArray("Encoder Angles", m_swerve.getModuleAngles());
    
    SmartDashboard.putNumber("F_Left Drive Vel", m_swerve.getFrontLeft().getDriveSpeed());
    SmartDashboard.putNumber("F_Right Drive Vel", m_swerve.getFrontRight().getDriveSpeed());
    SmartDashboard.putNumber("R_Left Drive Vel", m_swerve.getBackLeft().getDriveSpeed());
    SmartDashboard.putNumber("R_Right Drive Vel", m_swerve.getBackRight().getDriveSpeed());
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  @Override
  public void testPeriodic() {

    SmartDashboard.putData("F_Left Turn PID", m_swerve.getFrontLeft().getTurnPID());
    SmartDashboard.putData("F_Right Turn PID", m_swerve.getFrontRight().getTurnPID());
    SmartDashboard.putData("R_Left Turn PID", m_swerve.getBackLeft().getTurnPID());
    SmartDashboard.putData("R_Right Turn PID", m_swerve.getBackRight().getTurnPID());

  }

  private void driveWithJoystick(boolean fieldRelative) {
    
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(deadband(m_controller.getRawAxis(1), 0.04))
            * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(deadband(m_controller.getRawAxis(0), 0.04))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(deadband(m_controller.getRawAxis(2), 0.04))
            * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  public double deadband(double x, double db) {
    if(Math.abs(x) < db) {
      return 0;
    } else {
      return x;
    }
  }

}
