// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

public class Robot extends TimedRobot {
  private final Joystick m_controller = new Joystick(0);
  private final Drivetrain m_swerve = new Drivetrain();

  private Timer timer = new Timer();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void robotPeriodic() {
    
    SmartDashboard.putNumber("F_Left Drive Vel", m_swerve.getFrontLeft().getDriveSpeed());
    SmartDashboard.putNumber("F_Right Drive Vel", m_swerve.getFrontRight().getDriveSpeed());
    SmartDashboard.putNumber("R_Left Drive Vel", m_swerve.getBackLeft().getDriveSpeed());
    SmartDashboard.putNumber("R_Right Drive Vel", m_swerve.getBackRight().getDriveSpeed());

    SmartDashboard.putNumber("F_Left Turn Angle", m_swerve.getFrontLeft().getTurnAngle(m_swerve.getFrontLeft().getRawTurnAngle())*(180/Math.PI));
    SmartDashboard.putNumber("F_Right Turn Angle", m_swerve.getFrontRight().getTurnAngle(m_swerve.getFrontRight().getRawTurnAngle())*(180/Math.PI));
    SmartDashboard.putNumber("R_Left Turn Angle", m_swerve.getBackLeft().getTurnAngle(m_swerve.getBackLeft().getRawTurnAngle())*(180/Math.PI));
    SmartDashboard.putNumber("R_Right Turn Angle", m_swerve.getBackRight().getTurnAngle(m_swerve.getBackRight().getRawTurnAngle())*(180/Math.PI));

    SmartDashboard.putNumber("Front Left V Setpoint", m_swerve.getFrontLeft().getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Front Left A Setpoint", m_swerve.getFrontLeft().getTurnPID().getSetpoint().position*(180/Math.PI));

    SmartDashboard.putNumber("Front Right V Setpoint", m_swerve.getFrontRight().getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right A Setpoint", m_swerve.getFrontLeft().getTurnPID().getSetpoint().position*(180/Math.PI));

    SmartDashboard.putNumber("Back Left V Setpoint", m_swerve.getBackLeft().getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Back Left A Setpoint", m_swerve.getFrontLeft().getTurnPID().getSetpoint().position*(180/Math.PI));

    SmartDashboard.putNumber("Back Right V Setpoint", m_swerve.getBackRight().getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Back Right A Setpoint", m_swerve.getFrontLeft().getTurnPID().getSetpoint().position*(180/Math.PI));

    SmartDashboard.putNumber("Gyro", m_swerve.getGyroAngle());

    SmartDashboard.putNumber("Left Stick Y", m_controller.getRawAxis(1));
    SmartDashboard.putNumber("Left Stick X", m_controller.getRawAxis(0));
    SmartDashboard.putNumber("Right Stick X", m_controller.getRawAxis(2));
    
  }

  @Override
  public void autonomousPeriodic() {
    timer.start();
    if(timer.get() < 3) {
      m_swerve.drive(m_xspeedLimiter.calculate(0.5), 0, 0, true);
    } else {
      timer.stop();
      m_swerve.drive(0, 0, 0, false);
    }

    if(m_controller.getRawButton(1)) {
      timer.reset();
    }

  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
    if(m_controller.getRawButton(4)) { // triangle
      m_swerve.resetGyro();
      System.out.println("gyro reset!");
    }
  }

  @Override
  public void testPeriodic() {
  }

  private void driveWithJoystick(boolean fieldRelative) {
    
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(deadband(m_controller.getRawAxis(1), .06))
            * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(deadband(m_controller.getRawAxis(0), .06))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(deadband(m_controller.getRawAxis(2), .06))
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
