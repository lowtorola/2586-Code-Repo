package frc.robot;

import frc.robot.PID;
import frc.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  DifferentialDrive myRobot; // class that handles basic drive operations
  Joystick controller; // set to ID 1 in DriverStation

  PID autoPID_L;
  PID autoPID_R;

  double PID_Output_L;
  double PID_Output_R;

  /* The following PID Controller coefficients will need to be tuned */
  /* to match the dynamics of your drive system. Note that the */
  /* SmartDashboard in Test mode has support for helping you tune */
  /* controllers by displaying a form where you can enter new P, I, */
  /* and D constants and test the mechanism. */

  double autoDistance;

  static final double kToleranceInches = 1.0f;

  static final double kTargetInches = 60;

  final double kInchesPerRevolution = 12.56;

  final double gear_Ratio = 4.16;

  final double RAMP_RATE = 8;

  CANSparkMax leftMotor;
  CANSparkMax r_leftMotor;
  CANSparkMax rightMotor;
  CANSparkMax r_rightMotor;

  CANEncoder leftEncoder;
  CANEncoder rightEncoder;

  double leftEncDist;
  double rightEncDist;
  double leftEncRaw;
  double rightEncRaw;
  double leftError;
  double rightError;
  double avgError;

  @Override
  public void robotInit() {
    boolean leftMotorsInv = false;
    boolean rightMotorsInv = false;

    leftMotor = new CANSparkMax(3, MotorType.kBrushless);
    // leftMotor.setInverted(leftMotorsInv);

    r_leftMotor = new CANSparkMax(2, MotorType.kBrushless);
    // r_leftMotor.setInverted(leftMotorsInv);
    r_leftMotor.follow(leftMotor);

    rightMotor = new CANSparkMax(6, MotorType.kBrushless);
    // rightMotor.setInverted(false);

    r_rightMotor = new CANSparkMax(5, MotorType.kBrushless);
    r_rightMotor.follow(rightMotor);

    leftMotor.setIdleMode(IdleMode.kBrake);
    r_leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    r_rightMotor.setIdleMode(IdleMode.kBrake);

    leftEncoder = leftMotor.getEncoder();

    rightEncoder = rightMotor.getEncoder();

    myRobot = new DifferentialDrive(leftMotor, rightMotor);
    controller = new Joystick(0);

    autoPID_L = new PID(Constants.iP, Constants.iI, Constants.iD, Constants.DEADBAND);
    autoPID_R = new PID(Constants.iP, Constants.iI, Constants.iD, Constants.DEADBAND);

  }

  @Override
  public void autonomousInit() {
    
    leftMotor.setIdleMode(IdleMode.kBrake);
    r_leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    r_rightMotor.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void autonomousPeriodic() {
    
    sensorPrints();
    encoderZero();

    leftEncRaw = leftEncoder.getPosition();
    rightEncRaw = -rightEncoder.getPosition();
    leftEncDist = neoDistConv(leftEncRaw);
    rightEncDist = neoDistConv(rightEncRaw);
    leftError = kTargetInches - leftEncDist;
    rightError = kTargetInches - rightEncDist;
    avgError = (leftError + rightError) / 2;

    if (controller.getRawButton(1)) {
          
      leftMotor.setOpenLoopRampRate(RAMP_RATE);
      rightMotor.setOpenLoopRampRate(RAMP_RATE);
      PID_Output_L = autoPID_L.getOutput(leftError, 0);
      PID_Output_R = autoPID_R.getOutput(rightError, 0);

      double leftStickValue = -PID_Output_L;
      double rightStickValue = -PID_Output_R;
      myRobot.tankDrive(leftStickValue, rightStickValue);
    } else {
      leftMotor.set(0);
      rightMotor.set(0);
      leftMotor.setOpenLoopRampRate(3);
      rightMotor.setOpenLoopRampRate(3);
    }
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    encoderZero();
    leftEncRaw = leftEncoder.getPosition();
    rightEncRaw = -rightEncoder.getPosition();
    leftEncDist = neoDistConv(leftEncRaw);
    rightEncDist = neoDistConv(rightEncRaw);
    leftError = kTargetInches - leftEncDist;
    rightError = kTargetInches - rightEncDist;
    avgError = (leftError + rightError) / 2;

    if (controller.getRawButton(1)) {
          
      leftMotor.setOpenLoopRampRate(RAMP_RATE);
      rightMotor.setOpenLoopRampRate(RAMP_RATE);
      PID_Output_L = autoPID_L.getOutput(leftError, 0);
      PID_Output_R = autoPID_R.getOutput(rightError, 0);

      double leftStickValue = -PID_Output_L;
      double rightStickValue = -PID_Output_R;
      myRobot.tankDrive(leftStickValue, rightStickValue);
    } else {

      leftMotor.setOpenLoopRampRate(3);
      rightMotor.setOpenLoopRampRate(3);
      /* Standard tank drive, no driver assistance. */
      double drivePower = controller.getRawAxis(1);
      double driveRotate = controller.getRawAxis(2);
      
      myRobot.arcadeDrive(-drivePower, driveRotate);
      
      SmartDashboard.putNumber("drive Power", drivePower);
      SmartDashboard.putNumber("drive Rotate", driveRotate);

    }

      // Sensor Prints
      
      sensorPrints();

      // leftMotor.setIdleMode(IdleMode.kBrake);
      // r_leftMotor.setIdleMode(IdleMode.kBrake);
      // rightMotor.setIdleMode(IdleMode.kBrake);
      // r_rightMotor.setIdleMode(IdleMode.kBrake);
     /* leftMotor.setIdleMode(IdleMode.kCoast);
      r_leftMotor.setIdleMode(IdleMode.kCoast);
      rightMotor.setIdleMode(IdleMode.kCoast);
      r_rightMotor.setIdleMode(IdleMode.kCoast);
*/
    

    Timer.delay(0.005);

  }

  public double neoDistConv (double x) {
    x /= gear_Ratio;
    x *= kInchesPerRevolution;
    return x;
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    leftMotor.set(controller.getRawAxis(1));
   // r_leftMotor.set(controller.getRawAxis(1));
   // rightMotor.set(controller.getRawAxis(1));
   // r_rightMotor.set(controller.getRawAxis(1));
  }

  public void sensorPrints() {
    SmartDashboard.putNumber("error", avgError);
      SmartDashboard.putNumber("Front Left Output", leftMotor.getAppliedOutput());
      SmartDashboard.putNumber("Rear Left Output", r_leftMotor.getAppliedOutput());
      SmartDashboard.putNumber("Front Right Output", rightMotor.getAppliedOutput());
      SmartDashboard.putNumber("Rear Right Output", r_rightMotor.getAppliedOutput());
      SmartDashboard.putNumber("PID Output Left", PID_Output_L);
      SmartDashboard.putNumber("PID Output Right", PID_Output_R);
      SmartDashboard.putNumber("Left Encoder Distance", leftEncDist);
      SmartDashboard.putNumber("Right Encoder Distance", rightEncDist);
  }

  public void encoderZero() {
    if (controller.getRawButton(3)) {
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
    }
  }

}
