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

    SmartDashboard.putNumber("F_Left Turn Angle", m_swerve.getFrontLeft().getTurnAngle() * 180.0/Math.PI);
    SmartDashboard.putNumber("F_Right Turn Angle", m_swerve.getFrontRight().getTurnAngle() * 180.0/Math.PI);
    SmartDashboard.putNumber("R_Left Turn Angle", m_swerve.getBackLeft().getTurnAngle() * 180.0/Math.PI);
    SmartDashboard.putNumber("R_Right Turn Angle", m_swerve.getBackRight().getTurnAngle() * 180.0/Math.PI);

    SmartDashboard.putNumber("Front Left V Setpoint", m_swerve.getFrontLeft().getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Front Left A Setpoint", m_swerve.getFrontLeft().getState().angle.getDegrees());

    SmartDashboard.putNumber("Front Right V Setpoint", m_swerve.getFrontRight().getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right A Setpoint", m_swerve.getFrontRight().getState().angle.getDegrees());

    SmartDashboard.putNumber("Back Left V Setpoint", m_swerve.getBackLeft().getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Back Left A Setpoint", m_swerve.getBackLeft().getState().angle.getDegrees());

    SmartDashboard.putNumber("Back Right V Setpoint", m_swerve.getBackRight().getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Back Right A Setpoint", m_swerve.getBackRight().getState().angle.getDegrees());

    SmartDashboard.putNumber("Gyro", m_swerve.getGyroAngle());
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
    if(m_controller.getRawButton(4)) {
    }
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(false);
  }

  @Override
  public void testPeriodic() {
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
