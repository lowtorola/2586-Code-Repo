package frc.robot;

import frc.robot.PID;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  DifferentialDrive myRobot; // class that handles basic drive operations
  Joystick controller; // set to ID 1 in DriverStation

  PID limelightPID;

  double rotateToAngleRate;

  /* The following PID Controller coefficients will need to be tuned */
  /* to match the dynamics of your drive system. Note that the */
  /* SmartDashboard in Test mode has support for helping you tune */
  /* controllers by displaying a form where you can enter new P, I, */
  /* and D constants and test the mechanism. */

  double tx;

  static final double kToleranceDegrees = 2.0f;

  static final double kTargetAngleDegrees = 0.0f;

  CANSparkMax leftMotor;
  CANSparkMax r_leftMotor;
  CANSparkMax rightMotor;
  CANSparkMax r_rightMotor;

  @Override
  public void robotInit() {
    boolean leftMotorsInv = true;
    boolean rightMotorsInv = true;

    leftMotor = new CANSparkMax(3, MotorType.kBrushless);
    leftMotor.setInverted(leftMotorsInv);

    r_leftMotor = new CANSparkMax(2, MotorType.kBrushless);
    r_leftMotor.setInverted(leftMotorsInv);
    r_leftMotor.follow(leftMotor);

    rightMotor = new CANSparkMax(6, MotorType.kBrushless);
    rightMotor.setInverted(rightMotorsInv);

    r_rightMotor = new CANSparkMax(5, MotorType.kBrushless);
    r_rightMotor.setInverted(rightMotorsInv);
    r_rightMotor.follow(rightMotor);

    leftMotor.setIdleMode(IdleMode.kCoast);
    r_leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);
    r_rightMotor.setIdleMode(IdleMode.kCoast);

    myRobot = new DifferentialDrive(leftMotor, rightMotor);
    controller = new Joystick(0);

    final double iP = 0.036;
    // final double iI = 0.000035;
    final double iI = 0.000001;
    // final double iD = 0.115;
    final double iD = 0;
    final double DEADBAND = 0.1;

    limelightPID = new PID(iP, iI, iD, DEADBAND);

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

    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    myRobot.setSafetyEnabled(true);

    if (controller.getRawButton(1)) {

      leftMotor.setIdleMode(IdleMode.kCoast);
      r_leftMotor.setIdleMode(IdleMode.kCoast);
      rightMotor.setIdleMode(IdleMode.kCoast);
      r_rightMotor.setIdleMode(IdleMode.kCoast);

      rotateToAngleRate = limelightPID.getOutput(tx, 0);

      double leftStickValue = rotateToAngleRate;
      double rightStickValue = -rotateToAngleRate;
      myRobot.tankDrive(leftStickValue, rightStickValue);
    } else {

      /* Standard tank drive, no driver assistance. */
      double drive_Left = controller.getRawAxis(1);
      double drive_Right = -controller.getRawAxis(2);

      SmartDashboard.putNumber("drive Power", drive_Left);
      SmartDashboard.putNumber("drive Rotate", drive_Right);
      // Sensor Prints
      SmartDashboard.putNumber("error", tx);
      SmartDashboard.putNumber("Front Left Output", leftMotor.getAppliedOutput());
      SmartDashboard.putNumber("Rear Left Output", r_leftMotor.getAppliedOutput());
      SmartDashboard.putNumber("Front Right Output", rightMotor.getAppliedOutput());
      SmartDashboard.putNumber("Rear Right Output", r_rightMotor.getAppliedOutput());
      SmartDashboard.putNumber("Rotate To Angle Rate", rotateToAngleRate);

      myRobot.arcadeDrive(drive_Left, drive_Right, true);

      // leftMotor.setIdleMode(IdleMode.kBrake);
      // r_leftMotor.setIdleMode(IdleMode.kBrake);
      // rightMotor.setIdleMode(IdleMode.kBrake);
      // r_rightMotor.setIdleMode(IdleMode.kBrake);
      leftMotor.setIdleMode(IdleMode.kCoast);
      r_leftMotor.setIdleMode(IdleMode.kCoast);
      rightMotor.setIdleMode(IdleMode.kCoast);
      r_rightMotor.setIdleMode(IdleMode.kCoast);

    }
    Timer.delay(0.005);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
