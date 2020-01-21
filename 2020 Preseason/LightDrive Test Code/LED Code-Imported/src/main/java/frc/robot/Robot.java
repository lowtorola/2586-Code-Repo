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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.mach.LightDrive.*; //The LightDrive Library
import java.awt.Color; //Predefined colors and routines

public class Robot extends TimedRobot {

  private LightDriveCAN ld_can = new LightDriveCAN();

  Joystick m_stick;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    m_stick = new Joystick(0);

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

    if (light_switch.get()) {
      ld_can.SetColor(4, Color.green);
      /*
       * ld_can.SetColor(2, Color.yellow); ld_can.SetColor(3, new Color(1.0f, 0.5f,
       * 0.0f));
       */
      // Send latest colors to LightDrive
      ld_can.Update();
    } else {
      ld_can.SetColor(4, Color.red);

      ld_can.Update();

    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
