/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class Robot extends TimedRobot {

  private Joystick operatorStick;

  private Rev2mDistanceSensor distOnboard;

  private double onboardRange;

  @Override
  public void robotInit() {

    operatorStick = new Joystick(0);

    distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
    distOnboard.setAutomaticMode(true);

    onboardRange = 0;

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopPeriodic() {

    onboardRange = distOnboard.getRange();


    SmartDashboard.putNumber("Sensor Range:", onboardRange);

  }

  @Override
  public void testPeriodic() {
  }
}
