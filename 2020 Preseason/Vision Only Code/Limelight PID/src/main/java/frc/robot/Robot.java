package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {

  DifferentialDrive myRobot; // class that handles basic drive operations
  Joystick controller; // set to ID 1 in DriverStation
  AHRS ahrs;

  PIDController turnController;
  double rotateToAngleRate;

  /* The following PID Controller coefficients will need to be tuned */
  /* to match the dynamics of your drive system. Note that the */
  /* SmartDashboard in Test mode has support for helping you tune */
  /* controllers by displaying a form where you can enter new P, I, */
  /* and D constants and test the mechanism. */

  static final double kP = 0.0275;
  static final double kI = 0.000035;
  static final double kD = 0.115;
  static final double kF = 0;

  static final double kToleranceDegrees = 2.0f;

  static final double kTargetAngleDegrees = 0.0f;

  CANSparkMax leftMotor;
  CANSparkMax r_leftMotor;
  CANSparkMax rightMotor;
  CANSparkMax r_rightMotor;

  @Override
  public void robotInit() {
    leftMotor = new CANSparkMax(3, MotorType.kBrushless);
    leftMotor.setInverted(true);
    r_leftMotor = new CANSparkMax(2, MotorType.kBrushless);
    r_leftMotor.setInverted(true);
    r_leftMotor.follow(leftMotor);
    rightMotor = new CANSparkMax(6, MotorType.kBrushless);
    r_rightMotor = new CANSparkMax(5, MotorType.kBrushless);
    r_rightMotor.follow(rightMotor);

    leftMotor.setIdleMode(IdleMode.kCoast);
    r_leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);
    r_rightMotor.setIdleMode(IdleMode.kCoast);

    myRobot = new DifferentialDrive(leftMotor, rightMotor);
    myRobot.setExpiration(0.1);
    controller = new Joystick(0);
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
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
