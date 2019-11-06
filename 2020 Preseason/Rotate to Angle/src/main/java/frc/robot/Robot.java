package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the navX MXP to implement
 * the "rotate to angle", "zero yaw" and "drive straight" on a Tank
 * drive system.
 *
 * If Left Joystick Button 0 is pressed, a "turn" PID controller will 
 * set to point to a target angle, and while the button is held the drive
 * system will rotate to that angle (NOTE:  tank drive systems cannot simultaneously
 * move forward/reverse while rotating).
 *
 * This example also includes a feature allowing the driver to "reset"
 * the "yaw" angle.  When the reset occurs, the new gyro angle will be
 * 0 degrees.  This can be useful in cases when the gyro drifts, which
 * doesn't typically happen during a FRC match, but can occur during
 * long practice sessions.
 *
 * Finally, if Left Joystick button 2 is held, the "turn" PID controller will
 * be set to point to the current heading, and while the button is held,
 * the driver system will continue to point in the direction.  The robot 
 * can drive forward and backward (the magnitude of motion is the average
 * of the Y axis values on the left and right joysticks).
 *
 * Note that the PID Controller coefficients defined below will need to
 * be tuned for your drive system.
 */

public class Robot extends SampleRobot implements PIDOutput {
    DifferentialDrive myRobot;  // class that handles basic drive operations
    Joystick controller;  // set to ID 1 in DriverStation
    AHRS ahrs;

    PIDController turnController;
    double rotateToAngleRate;
    
    /* The following PID Controller coefficients will need to be tuned */
    /* to match the dynamics of your drive system.  Note that the      */
    /* SmartDashboard in Test mode has support for helping you tune    */
    /* controllers by displaying a form where you can enter new P, I,  */
    /* and D constants and test the mechanism.                         */
    
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

    public Robot() {

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
        try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
            ahrs = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
        turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-0.5, 0.5);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        turnController.disable();

    }
    
    /**
     * Runs the motors with tank steering.
     */
    
     void disabledInit() {
         leftMotor.setIdleMode(IdleMode.kCoast);
         r_leftMotor.setIdleMode(IdleMode.kCoast);
         rightMotor.setIdleMode(IdleMode.kCoast);
         r_rightMotor.setIdleMode(IdleMode.kCoast);
     }
    
     public void operatorControl() {


        myRobot.setSafetyEnabled(true);
        while (isOperatorControl() && isEnabled()) {
        	if ( controller.getRawButton(1)) {
        		/* While this button is held down, rotate to target angle.  
        		 * Since a Tank drive system cannot move forward simultaneously 
        		 * while rotating, all joystick input is ignored until this
        		 * button is released.
        		 */
        		if (!turnController.isEnabled()) {
        			turnController.setSetpoint(kTargetAngleDegrees);
        			rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
        			turnController.enable();
        		}
        		double leftStickValue = -rotateToAngleRate;
        		double rightStickValue = -rotateToAngleRate;
        		myRobot.tankDrive(leftStickValue,  rightStickValue);
        	} else if ( controller.getRawButton(2)) {
        		/* "Zero" the yaw (whatever direction the sensor is 
        		 * pointing now will become the new "Zero" degrees.
        		 */
        		ahrs.zeroYaw();
        	} else if ( controller.getRawButton(3)) {
        		/* While this button is held down, the robot is in
        		 * "drive straight" mode.  Whatever direction the robot
        		 * was heading when "drive straight" mode was entered
        		 * will be maintained.  The average speed of both 
        		 * joysticks is the magnitude of motion.
        		 */
        		if(!turnController.isEnabled()) {
        			// Acquire current yaw angle, using this as the target angle.
        			turnController.setSetpoint(ahrs.getYaw());
        			rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
        			turnController.enable();
        		}
        		double magnitude = (controller.getRawAxis(1) + controller.getRawAxis(5)) / 2;
        		double leftStickValue = magnitude + rotateToAngleRate;
        		double rightStickValue = magnitude - rotateToAngleRate;
        		myRobot.tankDrive(leftStickValue,  rightStickValue);
        	} else {
        		/* If the turn controller had been enabled, disable it now. */
        		if(turnController.isEnabled()) {
        			turnController.disable();
        		}
        		/* Standard tank drive, no driver assistance. */
        		double drive_Left = controller.getRawAxis(1);
            double drive_Right = -controller.getRawAxis(5);

            SmartDashboard.putNumber("drive Left", drive_Left);
            SmartDashboard.putNumber("drive Right", drive_Right);
            myRobot.setDeadband(0.2);
            myRobot.tankDrive(drive_Left, drive_Right, true);
        	}
            Timer.delay(0.005);		// wait for a motor update time

            statusPrints();
        }
    }

	@Override
    /* This function is invoked periodically by the PID Controller, */
    /* based upon navX MXP yaw angle input and PID Coefficients.    */
    public void pidWrite(double output) {
        rotateToAngleRate = output;
    }

    public void statusPrints() {
      SmartDashboard.putNumber("Gyro Yaw", ahrs.getYaw());
      SmartDashboard.putNumber("Left Output", leftMotor.getAppliedOutput());
      SmartDashboard.putNumber("Left Rear Output", r_leftMotor.getAppliedOutput());
      SmartDashboard.putNumber("Right Output", rightMotor.getAppliedOutput());
      SmartDashboard.putNumber("Rear Right Output", r_rightMotor.getAppliedOutput());
      SmartDashboard.putNumber("Rot to angle rate", rotateToAngleRate);
    }

}