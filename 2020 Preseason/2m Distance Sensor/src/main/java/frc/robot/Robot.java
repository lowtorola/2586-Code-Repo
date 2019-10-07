/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;


public class Robot extends TimedRobot {

  private Rev2mDistanceSensor distOnboard;

  @Override
  public void robotInit() {
    
  distOnboard = new Rev2mDistanceSensor(Port.kOnboard);

    
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

    distOnboard.setAutomaticMode(true);

    System.out.println(distOnboard.isRangeValid());
    

      System.out.println(distOnboard.getRange());
    

  }

  @Override
  public void testPeriodic() {
  }
}
