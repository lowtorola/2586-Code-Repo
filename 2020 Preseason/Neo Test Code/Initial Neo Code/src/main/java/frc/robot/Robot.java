
package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

public class Robot extends TimedRobot {

   DifferentialDrive m_drive;

   Joystick m_joystick = new Joystick(0);
   double drive_speed;
   double drive_rotate;

   CANSparkMax f_leftMotor;
   CANSparkMax r_leftMotor;
   CANSparkMax f_rightMotor;
   CANSparkMax r_rightMotor;

  @Override
  public void robotInit() {

    drive_speed = -1 * m_joystick.getRawAxis(1);
    drive_rotate = m_joystick.getRawAxis(2);

    f_leftMotor = new CANSparkMax(3, MotorType.kBrushless);
    r_leftMotor = new CANSparkMax(5, MotorType.kBrushless);
    f_rightMotor = new CANSparkMax(6, MotorType.kBrushless);
    r_rightMotor = new CANSparkMax(2, MotorType.kBrushless);

    f_leftMotor.setSmartCurrentLimit(40);
    r_leftMotor.setSmartCurrentLimit(40);
    f_rightMotor.setSmartCurrentLimit(40);
    r_rightMotor.setSmartCurrentLimit(40);

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

    m_drive.arcadeDrive(drive_speed, drive_rotate, true);


  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
