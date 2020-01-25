
package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  DifferentialDrive m_drive;

  Joystick m_joystick = new Joystick(0);
  double drive_speed;
  double drive_rotate;

  CANSparkMax leftMaster;
  CANSparkMax leftSlave;
  CANSparkMax rightMaster;
  CANSparkMax rightSlave;

  private CANEncoder f_leftEncoder;
  private CANEncoder f_rightEncoder;

  Encoder leftEncoder;
  Encoder rightEncoder;

  AHRS gyro;
  double gyroYaw;
  double gyroVelocity;

  // private Rev2mDistanceSensor distOnboard;
  private double onboardRange;
  private double rangeAdjust;

  Compressor comp;

  DoubleSolenoid shifter;

  @Override
  public void robotInit() {

    boolean leftMotorsInverted = false;
    boolean rightMotorsInverted = false;

    leftMaster = new CANSparkMax(3, MotorType.kBrushless);
    leftMaster.setInverted(leftMotorsInverted);
    leftSlave = new CANSparkMax(2, MotorType.kBrushless);
    leftSlave.setInverted(leftMotorsInverted);
    rightMaster = new CANSparkMax(6, MotorType.kBrushless);
    rightMaster.setInverted(rightMotorsInverted);
    rightSlave = new CANSparkMax(5, MotorType.kBrushless);
    rightSlave.setInverted(rightMotorsInverted);

    leftMaster.setIdleMode(IdleMode.kBrake);
    leftSlave.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave.setIdleMode(IdleMode.kBrake);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftEncoder = new Encoder(6, 7);
    rightEncoder = new Encoder(8, 9);
    f_leftEncoder = leftMaster.getEncoder();
    f_rightEncoder = rightMaster.getEncoder();

    double kPulsesPerRevolution = 1440;
    double kInchesPerRevolution = 12.5;
    double kInchesPerPulse = kInchesPerRevolution / kPulsesPerRevolution;

    leftEncoder.setDistancePerPulse(0.008680555555555556);
    leftEncoder.setSamplesToAverage(5);

    rightEncoder.setDistancePerPulse(0.008680555555555556);
    rightEncoder.setReverseDirection(true);
    rightEncoder.setSamplesToAverage(5);

    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();

    // distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
    // distOnboard.setAutomaticMode(true);
    onboardRange = 0;

    // comp = new Compressor();
    // comp.start();

    shifter = new DoubleSolenoid(6, 7);

    m_drive = new DifferentialDrive(leftMaster, rightMaster);

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    if (m_joystick.getRawButton(1)) {
      leftEncoder.reset();
      rightEncoder.reset();
      f_leftEncoder.setPosition(0);
      f_rightEncoder.setPosition(0);
    }

    if (m_joystick.getRawButton(2)) {
      gyro.reset();
    }

    shifting();
    motorStatus();
    sensorPrint();
    visionAim();

  }

  public void driveBase() {

    drive_speed = m_joystick.getRawAxis(1);
    drive_rotate = m_joystick.getRawAxis(2);

    m_drive.arcadeDrive(drive_speed, drive_rotate, true);

  }

  public void shifting() {
    if (m_joystick.getRawButton(5)) {
      shifter.set(DoubleSolenoid.Value.kForward);
    } else if (m_joystick.getRawButton(6)) {
      shifter.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void motorStatus() {
    SmartDashboard.putNumber("f_left Voltage", leftMaster.getBusVoltage());
    SmartDashboard.putNumber("r_left Voltage", leftSlave.getBusVoltage());
    SmartDashboard.putNumber("f_right Voltage", rightMaster.getBusVoltage());
    SmartDashboard.putNumber("r_right Voltage", rightSlave.getBusVoltage());

    SmartDashboard.putNumber("f_left Temperature", leftMaster.getMotorTemperature());
    SmartDashboard.putNumber("r_left Temperature", leftSlave.getMotorTemperature());
    SmartDashboard.putNumber("f_right Temperature", rightMaster.getMotorTemperature());
    SmartDashboard.putNumber("r_right Temperature", rightMaster.getMotorTemperature());

    SmartDashboard.putNumber("f_Left Output", leftMaster.getAppliedOutput());
    SmartDashboard.putNumber("r_Left Output", leftSlave.getAppliedOutput());
    SmartDashboard.putNumber("f_Right Output", rightMaster.getAppliedOutput());
    SmartDashboard.putNumber("r_Right Output", rightMaster.getAppliedOutput());

  }

  public void sensorPrint() {

    gyroYaw = gyro.getYaw();
    gyroVelocity = convertVelocity(gyro.getVelocityX());

    SmartDashboard.putNumber("left encoder raw", leftEncoder.getRaw());
    SmartDashboard.putNumber("right encoder raw", rightEncoder.getRaw());

    SmartDashboard.putNumber("left encoder rate", leftEncoder.getRate());
    SmartDashboard.putNumber("right encoder rate", rightEncoder.getRate());

    SmartDashboard.putNumber("left encoder distance", leftEncoder.getDistance());
    SmartDashboard.putNumber("right encoder distance", rightEncoder.getDistance());

    SmartDashboard.putNumber("left NEO encoder rotations", f_leftEncoder.getPosition());
    SmartDashboard.putNumber("right NEO encoder rotations", f_rightEncoder.getPosition());

    SmartDashboard.putNumber("Gyro Angle", gyroYaw);
    SmartDashboard.putNumber("Robot Velocity", gyroVelocity);
    SmartDashboard.putNumber("Sensor Range:", onboardRange);

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    /*
     * if (m_joystick.getRawButton(1)) { leftMaster.set(-m_joystick.getRawAxis(1));
     * } else if (m_joystick.getRawButton(2)) {
     * leftSlave.set(-m_joystick.getRawAxis(1)); }
     */
    /*
     * if (m_joystick.getRawButton(3)) { rightSlave.set(m_joystick.getRawAxis(1));
     * } else { rightSlave.set(0); }
     */
    /*
     * if (m_joystick.getRawButton(4)) { rightMaster.set(m_joystick.getRawAxis(1));
     * } else { rightMaster.set(0); }
     */

  }

  public double convertVelocity(double x) {
    x *= 3.28;
    return x;
  }

  public void visionAim() {

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    // System.out.println(tx);

    double drive_power = -m_joystick.getRawAxis(1);
    double drive_rotate = m_joystick.getRawAxis(2);
    boolean autoAim = m_joystick.getRawButton(1);

    // onboardRange = distOnboard.getRange();

    double leftCommand = 0;
    double rightCommand = 0;

    float Kp = -0.02f;
    float min_command = 0.07f;

    if (m_joystick.getRawButton(1)) {
      float heading_error = (float) -tx;
      float steering_adjust = 0.0f;
      if (tx > 1.0) {
        steering_adjust = Kp * heading_error - min_command;
      } else if (tx < 1.0) {
        steering_adjust = Kp * heading_error + min_command;
      }

      /*
       * if (onboardRange > 10 && onboardRange < 200) { rangeAdjust = onboardRange *
       * Kp -min_command; } else { rangeAdjust = 0;
       * System.out.println("Warning! Range is too far!"); }
       */
      leftCommand += steering_adjust;
      rightCommand -= steering_adjust;

      leftMaster.setIdleMode(IdleMode.kCoast);
      leftSlave.setIdleMode(IdleMode.kCoast);
      rightMaster.setIdleMode(IdleMode.kCoast);
      rightSlave.setIdleMode(IdleMode.kCoast);

      System.out.println(steering_adjust);
      SmartDashboard.putNumber("Left Vision Drive", leftCommand);
      SmartDashboard.putNumber("Right Vision Drive", rightCommand);
      m_drive.tankDrive(leftCommand, rightCommand);
    } else {
      m_drive.arcadeDrive(drive_power, drive_rotate, true);
    }

  }

}
