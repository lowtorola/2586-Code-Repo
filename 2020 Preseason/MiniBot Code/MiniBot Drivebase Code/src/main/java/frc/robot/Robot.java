
package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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

   CANSparkMax f_leftMotor;
   CANSparkMax r_leftMotor;
   CANSparkMax f_rightMotor;
   CANSparkMax r_rightMotor;

   Encoder leftEncoder;
   Encoder rightEncoder;

   AHRS gyro;

   Compressor comp;

   DoubleSolenoid shifter;

  @Override
  public void robotInit() {


    f_leftMotor = new CANSparkMax(3, MotorType.kBrushless);
    r_leftMotor = new CANSparkMax(2, MotorType.kBrushless);
    f_rightMotor = new CANSparkMax(6, MotorType.kBrushless);
    r_rightMotor = new CANSparkMax(5, MotorType.kBrushless);

    f_leftMotor.setIdleMode(IdleMode.kBrake);
    r_leftMotor.setIdleMode(IdleMode.kBrake);
    f_rightMotor.setIdleMode(IdleMode.kBrake);
    r_rightMotor.setIdleMode(IdleMode.kBrake);

    leftEncoder = new Encoder(6, 7);
    rightEncoder = new Encoder(8, 9);

    
    double kPulsesPerRevolution = 360;
		double kInchesPerRevolution = 12.5;
		double kInchesPerPulse = kInchesPerRevolution / kPulsesPerRevolution;
   
    leftEncoder.setDistancePerPulse(kInchesPerPulse);
    leftEncoder.setSamplesToAverage(3);
    
    rightEncoder.setDistancePerPulse(kInchesPerPulse);
    rightEncoder.setReverseDirection(true);
    rightEncoder.setSamplesToAverage(3);

    gyro = new AHRS(SPI.Port.kMXP);
    gyro.calibrate();
    gyro.reset();

    // comp = new Compressor();
    // comp.start();

    shifter = new DoubleSolenoid(6, 7);

    SpeedControllerGroup leftMotors = new SpeedControllerGroup(f_leftMotor, r_leftMotor);
    SpeedControllerGroup rightMotors = new SpeedControllerGroup(f_rightMotor, r_rightMotor);

    m_drive = new DifferentialDrive(leftMotors, rightMotors);

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

    if(m_joystick.getRawButton(1)) {
      leftEncoder.reset();
      rightEncoder.reset();
    }

    if(m_joystick.getRawButton(2)) {
      gyro.reset();
    }

  
    driveBase();
    shifting();
    motorStatus();
    sensorPrint();

  }

  public void driveBase() {

    drive_speed = -1 * m_joystick.getRawAxis(1);
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
    SmartDashboard.putNumber("f_left Voltage", f_leftMotor.getBusVoltage());
    SmartDashboard.putNumber("r_left Voltage", r_leftMotor.getBusVoltage());
    SmartDashboard.putNumber("f_right Voltage", f_rightMotor.getBusVoltage());
    SmartDashboard.putNumber("r_right Voltage", r_rightMotor.getBusVoltage());

    SmartDashboard.putNumber("f_left Temperature", f_leftMotor.getMotorTemperature());
    SmartDashboard.putNumber("r_left Temperature", r_leftMotor.getMotorTemperature());
    SmartDashboard.putNumber("f_right Temperature", f_rightMotor.getMotorTemperature());
    SmartDashboard.putNumber("r_right Temperature", r_rightMotor.getMotorTemperature());

    SmartDashboard.putNumber("f_Left Output", f_leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("r_Left Output", r_leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("f_Right Output", f_rightMotor.getAppliedOutput());
    SmartDashboard.putNumber("r_Right Output", r_rightMotor.getAppliedOutput());

  }

  public void sensorPrint() {

    SmartDashboard.putNumber("left encoder raw", leftEncoder.getRaw());
    SmartDashboard.putNumber("right encoder raw", rightEncoder.getRaw());

    SmartDashboard.putNumber("left encoder rate", leftEncoder.getRate());
    SmartDashboard.putNumber("right encoder rate", rightEncoder.getRate());

    SmartDashboard.putNumber("left encoder distance", leftEncoder.getDistance());
    SmartDashboard.putNumber("right encoder distance", rightEncoder.getDistance());

    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    SmartDashboard.putNumber("Robot Velocity", gyro.getVelocityX());

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  /*  if (m_joystick.getRawButton(1)) {
      f_leftMotor.set(-m_joystick.getRawAxis(1));
    }
    else if (m_joystick.getRawButton(2)) {
      r_leftMotor.set(-m_joystick.getRawAxis(1));
    } */
    /* if (m_joystick.getRawButton(3)) {
      r_rightMotor.set(m_joystick.getRawAxis(1));
     } else {
       r_rightMotor.set(0);
     } */
    /* if (m_joystick.getRawButton(4)) {
      f_rightMotor.set(m_joystick.getRawAxis(1));
    } else {
      f_rightMotor.set(0);
    } */
    
  }
}
