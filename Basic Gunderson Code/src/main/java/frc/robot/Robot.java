/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.mach.LightDrive.LightDrivePWM;
import java.awt.Color;



  public class Robot extends TimedRobot {
  
  Joystick operatorController, driverController;

  // LightDrive 
  LightDrivePWM ldrive_pwm;
  Servo servo1;
	Servo servo2;

  // Camera
  CameraServer cameraServer;

  // Elevator elevatorWinch
  WPI_TalonSRX elevatorWinch, frontLeft, rearLeft, frontRight, rearRight;
  DifferentialDrive mainDrive;
  double d_power, d_rotation;

  // HP Mech
  Compressor comp;
  DoubleSolenoid mechUpDown, hatchGrabRelease, HABClimbFront;

  // HAB Lift
  Spark HABDrive;
  DigitalInput HABLiftHigh;
  

  @Override
  public void robotInit() {
    
    driverController = new Joystick(0);
    operatorController = new Joystick(1);

    // Pneumatics
    comp = new Compressor();
   // comp.start();
    mechUpDown = new DoubleSolenoid(0, 1);
    hatchGrabRelease = new DoubleSolenoid(2, 3);
    HABClimbFront = new DoubleSolenoid(4, 5);

    // Camera
    CameraServer.getInstance().startAutomaticCapture();
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(1200, 900);
    camera.setFPS(30);

    //Initialize a new PWM LightDrive
    //Initialize 2 Servo outputs for a PWM LightDrive
		servo1 = new Servo(3);
		servo2 = new Servo(4);
    ldrive_pwm = new LightDrivePWM(servo1, servo2);

    // Elevator winch
    elevatorWinch = new WPI_TalonSRX(5);

    // Drive Motors
    frontLeft = new WPI_TalonSRX(1);
    rearLeft = new WPI_TalonSRX(2);
    frontRight = new WPI_TalonSRX(4);
    rearRight = new WPI_TalonSRX(3);
    SpeedControllerGroup left = new SpeedControllerGroup(frontLeft, rearLeft);
    SpeedControllerGroup right = new SpeedControllerGroup(frontRight, rearRight);

    mainDrive = new DifferentialDrive(left, right);

    // HAB Lift
    HABDrive = new Spark(2);
    HABLiftHigh = new DigitalInput(4);

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
    auxilaries();

    d_power = squareInput(deadZoneComp(driverController.getRawAxis(1) * -1));
    d_rotation = squareInput(deadZoneComp(driverController.getRawAxis(2)));

    mainDrive.arcadeDrive(d_power, d_rotation);

  }

  public void auxilaries() { // Method containing all mechs except for elevator and intake

    // LEDs
    if (operatorController.getRawButton(2)) {
      ldrive_pwm.SetColor(1, Color.RED);
    }

    // Elevator will stop when it hits the bottom limit

    double liftCommand = deadZoneComp(operatorController.getRawAxis(1) * -1); // Left stick Y
  
    elevatorWinch.set(liftCommand);

    // Hatch Mech
    if (operatorController.getRawButton(10)) {
      mechUpDown.set(DoubleSolenoid.Value.kForward); // Options button
    } else if (operatorController.getRawButton(9)) {
      mechUpDown.set(DoubleSolenoid.Value.kReverse); // Share button
    }

    if (operatorController.getRawButton(5)) {
      hatchGrabRelease.set(DoubleSolenoid.Value.kForward); // left bumper
    } else if (operatorController.getRawButton(6)) {
      hatchGrabRelease.set(DoubleSolenoid.Value.kReverse); // right bumper
    }

    // HAB Climb Front
    if (operatorController.getPOV() == 180) { // down POV
      HABClimbFront.set(DoubleSolenoid.Value.kForward);
    } else if (operatorController.getPOV() == 0) { // up POV
      HABClimbFront.set(DoubleSolenoid.Value.kReverse);
    }

    // HAB Lift
    boolean HABLiftTop = HABLiftHigh.get(); // The "high limit switch" is to stop it from going too low. Sounds wrong
                                            // but should be right
    // boolean HABLiftBottom = HABLiftLow.get();
    double HABWinchDrive = deadZoneComp(operatorController.getRawAxis(5) * -1); // Right stick Y

    if (operatorController.getRawButton(4)) {

      if (HABWinchDrive < 0.0) {
        HABDrive.set(HABWinchDrive);
      } else if (HABWinchDrive > 0.0 && HABLiftTop) {
        HABDrive.set(HABWinchDrive);
      } else {
        HABDrive.set(0.0);
      }

    } else {
      HABDrive.set(0.0);
    }
  }

  public static CameraServer getCameraServer() {
    return CameraServer.getInstance();
  }

  // Just a simple dead zone
  public double deadZoneComp(double x) {
    if (Math.abs(x) < 0.08) {
      return 0;
    } else {
      return x;
    }
  }

  // Squares input: for ease-of-driving
  public double squareInput(double input) {
    if (input < 0) {
      return (-1 * (Math.pow(input, 2)));
    } else if (input > 0) {
      return (Math.pow(input, 2));
     
    } else {
      return 0;
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
