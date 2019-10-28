/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private CANSparkMax f_leftMotor;
  private CANSparkMax f_rightMotor;

  private final double ROTS_PER_DEGREE = 1 / 12.86; // rotations per degree

  private CANPIDController left_pidController;
  private CANPIDController right_pidController;

  private CANEncoder f_leftEncoder;
  private CANEncoder f_rightEncoder;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private Joystick m_joystick;

  @Override
  public void robotInit() {

    m_joystick = new Joystick(0);

    // initialize motor
    f_leftMotor = new CANSparkMax(3, MotorType.kBrushless);
    f_rightMotor = new CANSparkMax(6, MotorType.kBrushless);

    f_leftMotor.setIdleMode(IdleMode.kCoast);
    f_rightMotor.setIdleMode(IdleMode.kCoast);

    /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    left_pidController = f_leftMotor.getPIDController();
    right_pidController = f_rightMotor.getPIDController();

    // Encoder object created to display position values
    f_leftEncoder = f_leftMotor.getEncoder();
    f_rightEncoder = f_rightMotor.getEncoder();

    // PID coefficients
    kP = .3;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    left_pidController.setP(kP);
    left_pidController.setI(kI);
    left_pidController.setD(kD);
    left_pidController.setIZone(kIz);
    left_pidController.setFF(kFF);
    left_pidController.setOutputRange(kMinOutput, kMaxOutput);

    right_pidController.setP(kP);
    right_pidController.setI(kI);
    right_pidController.setD(kD);
    right_pidController.setIZone(kIz);
    right_pidController.setFF(kFF);
    right_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }

  @Override
  public void teleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);
    SmartDashboard.putNumber("Left Output", f_leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("Right Output", f_rightMotor.getAppliedOutput());

    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double x_ref = degreesToRotations(-tx);

    SmartDashboard.putNumber("X Error Degrees", tx);
    SmartDashboard.putNumber("X Error Rotations", x_ref);

    if (m_joystick.getRawButton(4)) {
      left_pidController.setReference(x_ref, ControlType.kPosition);
      right_pidController.setReference(-x_ref, ControlType.kPosition);
    }
    SmartDashboard.putNumber("SetPoint", x_ref);
    SmartDashboard.putNumber("LeftProcessVariable", f_leftEncoder.getPosition());
    SmartDashboard.putNumber("RightProcessVariable", f_rightEncoder.getPosition());

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kP)) {
      left_pidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      left_pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      left_pidController.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      left_pidController.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      left_pidController.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      left_pidController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;

      if ((p != kP)) {
        right_pidController.setP(p);
        kP = p;
      }
      if ((i != kI)) {
        right_pidController.setI(i);
        kI = i;
      }
      if ((d != kD)) {
        right_pidController.setD(d);
        kD = d;
      }
      if ((iz != kIz)) {
        right_pidController.setIZone(iz);
        kIz = iz;
      }
      if ((ff != kFF)) {
        right_pidController.setFF(ff);
        kFF = ff;
      }
      if ((max != kMaxOutput) || (min != kMinOutput)) {
        right_pidController.setOutputRange(min, max);
        kMinOutput = min;
        kMaxOutput = max;

      }

      /**
       * PIDController objects are commanded to a set point using the SetReference()
       * method.
       * 
       * The first parameter is the value of the set point, whose units vary depending
       * on the control type set in the second parameter.
       * 
       * The second parameter is the control type can be set to one of four
       * parameters: com.revrobotics.ControlType.kDutyCycle
       * com.revrobotics.ControlType.kPosition com.revrobotics.ControlType.kVelocity
       * com.revrobotics.ControlType.kVoltage
       */

      
    }
  }

  public double degreesToRotations(double x) {

    x *= ROTS_PER_DEGREE;

    return x;

  }

}
