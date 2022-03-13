/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

/**
 * Before Running:
 * Open shuffleBoard, select File->Load Layout and select the 
 * shuffleboard.json that is in the root directory of this example
 */

/**
 * REV Smart Motion Guide
 * 
 * The SPARK MAX includes a new control mode, REV Smart Motion which is used to 
 * control the position of the motor, and includes a max velocity and max 
 * acceleration parameter to ensure the motor moves in a smooth and predictable 
 * way. This is done by generating a motion profile on the fly in SPARK MAX and 
 * controlling the velocity of the motor to follow this profile.
 * 
 * Since REV Smart Motion uses the velocity to track a profile, there are only 
 * two steps required to configure this mode:
 *    1) Tune a velocity PID loop for the mechanism
 *    2) Configure the smart motion parameters
 * 
 * Tuning the Velocity PID Loop
 * 
 * The most important part of tuning any closed loop control such as the velocity 
 * PID, is to graph the inputs and outputs to understand exactly what is happening. 
 * For tuning the Velocity PID loop, at a minimum we recommend graphing:
 *
 *    1) The velocity of the mechanism (‘Process variable’)
 *    2) The commanded velocity value (‘Setpoint’)
 *    3) The applied output
 *
 * This example will use ShuffleBoard to graph the above parameters. Make sure to
 * load the shuffleboard.json file in the root of this directory to get the full
 * effect of the GUI layout.
 */
public class Robot extends TimedRobot {
  private static final int LEFT_MOTOR = 14; // 14 port, 15 starboard
  private static final int RIGHT_MOTOR = 15;
  private CANSparkMax m_leftMotor, m_rightMotor;
  private SparkMaxPIDController m_leftPidController, m_rightPidController;
  private RelativeEncoder m_leftEncoder, m_rightEncoder;
  public double kP_Left, kI_Left, kD_Left, kIz_Left, kFF_Left, kMaxOutput_Left, kMinOutput_Left, maxRPM_Left, maxVel_Left, minVel_Left, maxAcc_Left, allowedErr_Left;
  public double kP_Right, kI_Right, kD_Right, kIz_Right, kFF_Right, kMaxOutput_Right, kMinOutput_Right, maxRPM_Right, maxVel_Right, minVel_Right, maxAcc_Right, allowedErr_Right;

  private Joystick m_controller;

  @Override
  public void robotInit() {
    // initialize motor
    m_leftMotor = new CANSparkMax(LEFT_MOTOR, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(RIGHT_MOTOR, MotorType.kBrushless);
    m_rightMotor.setInverted(true);
    m_leftMotor.setSmartCurrentLimit(40);
    m_rightMotor.setSmartCurrentLimit(40);

    m_controller = new Joystick(0); // DS4

    // initialize Left PID controller and encoder objects
    m_leftPidController = m_leftMotor.getPIDController();
    m_leftEncoder = m_leftMotor.getEncoder();
    m_leftEncoder.setPosition(0);
    // PID coefficients
    kP_Left = .000175; 
    kI_Left = 0;
    kD_Left = 0; 
    kIz_Left = 0; 
    kFF_Left = .000295;
    kMaxOutput_Left = .8; 
    kMinOutput_Left = -.9;
    maxRPM_Left = 5700;

    // Smart Motion Coefficients
    maxVel_Left = 3000; // rpm
    maxAcc_Left = 1600;

    // set PID coefficients
    m_leftPidController.setP(kP_Left);
    m_leftPidController.setI(kI_Left);
    m_leftPidController.setD(kD_Left);
    m_leftPidController.setIZone(kIz_Left);
    m_leftPidController.setFF(kFF_Left);
    m_leftPidController.setOutputRange(kMinOutput_Left, kMaxOutput_Left);

        // initialize PID controller and encoder objects
    m_rightPidController = m_rightMotor.getPIDController();
    m_rightEncoder = m_rightMotor.getEncoder();
    m_rightEncoder.setPosition(0);
    // PID coefficients
    // FIXME: Get gains from sysid!!
    kP_Right = .000195; 
    kI_Right = 0;
    kD_Right = 0; 
    kIz_Right = 0; 
    kFF_Right = .000305;
    kMaxOutput_Right = .8; 
    kMinOutput_Right = -.9;
    maxRPM_Right = 5700;

    // Smart Motion Coefficients
    maxVel_Right = 3000; // rpm
    maxAcc_Right = 1600;

    // set PID coefficients
    m_rightPidController.setP(kP_Right);
    m_rightPidController.setI(kI_Right);
    m_rightPidController.setD(kD_Right);
    m_rightPidController.setIZone(kIz_Right);
    m_rightPidController.setFF(kFF_Right);
    m_rightPidController.setOutputRange(kMinOutput_Right, kMaxOutput_Right);

    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;

    m_leftPidController.setSmartMotionMaxVelocity(maxVel_Left, smartMotionSlot);
    m_leftPidController.setSmartMotionMinOutputVelocity(minVel_Left, smartMotionSlot);
    m_leftPidController.setSmartMotionMaxAccel(maxAcc_Left, smartMotionSlot);
    m_leftPidController.setSmartMotionAllowedClosedLoopError(allowedErr_Left, smartMotionSlot);

    m_rightPidController.setSmartMotionMaxVelocity(maxVel_Right, smartMotionSlot);
    m_rightPidController.setSmartMotionMinOutputVelocity(minVel_Right, smartMotionSlot);
    m_rightPidController.setSmartMotionMaxAccel(maxAcc_Right, smartMotionSlot);
    m_rightPidController.setSmartMotionAllowedClosedLoopError(allowedErr_Right, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP_Right);
    SmartDashboard.putNumber("I Gain", kI_Right);
    SmartDashboard.putNumber("D Gain", kD_Right);
    SmartDashboard.putNumber("I Zone", kIz_Right);
    SmartDashboard.putNumber("Feed Forward", kFF_Right);
    SmartDashboard.putNumber("Max Output", kMaxOutput_Right);
    SmartDashboard.putNumber("Min Output", kMinOutput_Right);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel_Right);
    SmartDashboard.putNumber("Min Velocity", minVel_Right);
    SmartDashboard.putNumber("Max Acceleration", maxAcc_Right);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr_Right);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

  }

  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("Left Position", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Position", m_rightEncoder.getPosition());

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP_Right)) { m_rightPidController.setP(p); kP_Right = p; }
    if((i != kI_Right)) { m_rightPidController.setI(i); kI_Right = i; }
    if((d != kD_Right)) { m_rightPidController.setD(d); kD_Right = d; }
    if((iz != kIz_Right)) { m_rightPidController.setIZone(iz); kIz_Right = iz; }
    if((ff != kFF_Right)) { m_rightPidController.setFF(ff); kFF_Right = ff; }
    if((max != kMaxOutput_Right) || (min != kMinOutput_Right)) { 
      m_rightPidController.setOutputRange(min, max); 
      kMinOutput_Right = min; kMaxOutput_Right = max; 
    }
    if((maxV != maxVel_Right)) { m_rightPidController.setSmartMotionMaxVelocity(maxV,0); maxVel_Right = maxV; }
    if((minV != minVel_Right)) { m_rightPidController.setSmartMotionMinOutputVelocity(minV,0); minVel_Right = minV; }
    if((maxA != maxAcc_Right)) { m_rightPidController.setSmartMotionMaxAccel(maxA,0); maxAcc_Right = maxA; }
    if((allE != allowedErr_Right)) { m_rightPidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr_Right = allE; }

    if(m_controller.getRawButton(4)) { // top of 4 buttons
      m_leftPidController.setReference(60, CANSparkMax.ControlType.kSmartMotion);
      m_rightPidController.setReference(60, CANSparkMax.ControlType.kSmartMotion);

    } else if(m_controller.getRawButton(2)) { // bottom of 4 buttons
      m_leftPidController.setReference(10, CANSparkMax.ControlType.kSmartMotion);
      m_rightPidController.setReference(10, CANSparkMax.ControlType.kSmartMotion);

    // } else if(m_controller.getRawButton(14)) { //center pad
    //   m_leftPidController.setSmartMotionMaxVelocity(100, 0);
    //   m_rightPidController.setSmartMotionMaxVelocity(100, 0);
    //   m_leftPidController.setReference(-30, CANSparkMax.ControlType.kSmartMotion);
    //   m_rightPidController.setReference(-30, CANSparkMax.ControlType.kSmartMotion);
    //   m_leftPidController.setSmartMotionMaxVelocity(maxVel_Left, 0);
    //   m_rightPidController.setSmartMotionMaxVelocity(maxVel_Right, 0);
    // }

    // if(!m_leftMotor.getReverseLimitSwitch(Type.kNormallyClosed) && m_leftMotor.getAppliedOutput() < 0) {
    //   m_leftEncoder.setPosition(0);
    //   m_leftPidController.setReference(0, CANSparkMax.ControlType.kSmartMotion);
    // }

    // if(!m_rightLowLimit.get() && m_rightMotor.getAppliedOutput() < 0) {
    //   m_rightEncoder.setPosition(0);
    //   m_rightPidController.setReference(0, CANSparkMax.ControlType.kSmartMotion);
    // }



    // if(mode) {
    //   setPoint = SmartDashboard.getNumber("Set Velocity", 0);
    //   m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    //   processVariable = m_encoder.getVelocity();
    // } else {
    //   setPoint = SmartDashboard.getNumber("Set Position", 0);
    //   /**
    //    * As with other PID modes, Smart Motion is set by calling the
    //    * setReference method on an existing pid object and setting
    //    * the control type to kSmartMotion
    //    */
    //   m_pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
    //   processVariable = m_encoder.getPosition();
    // }
    
    // SmartDashboard.putNumber("SetPoint", setPoint);
    // SmartDashboard.putNumber("Process Variable", processVariable);
    // SmartDashboard.putNumber("Output", m_motor.getAppliedOutput());
  }
}
}