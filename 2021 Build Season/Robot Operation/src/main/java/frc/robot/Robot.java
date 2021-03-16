/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.ShooterPreload;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

  // Adding a comment to test 2021 software!!

public class Robot extends TimedRobot {

  private RobotContainer robotContainer;

  private Command autonomousCommand;

  Joystick m_controller = new Joystick(1);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    robotContainer.periodic();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

   DriveSubsystem m_drive = new DriveSubsystem();

    m_drive.setDeadband(DriveConstants.kDriveDeadband);
    // drive controls
    m_drive.setDefaultCommand(
      new DefaultDrive(
        m_drive,
          () -> -m_controller.getRawAxis(1),
          () -> m_controller.getRawAxis(2)));
    configureTestBindings();

  }

  public void configureTestBindings() {
/*
   ShooterSubsystem m_shooter = new ShooterSubsystem();
   IndexerSubsystem m_indexer = new IndexerSubsystem();
   IntakeSubsystem m_intake = new IntakeSubsystem();

        // shooter spool up
        new JoystickButton(m_controller, OIConstants.kCircleButton) // O
        .whenHeld(new PIDCommand( 
          new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD),
          // Close the loop on Shooter Current RPM
          m_shooter::getMeasurement,
          // get setpoint
          2500,
          // pipe the output to the shooter motor
          output -> m_shooter.runShooter((output / ShooterConstants.kMaxRPM) + m_shooter.getFeedForward())), true)
          .whenReleased(new InstantCommand(m_shooter::stopShooter, m_shooter)); 
  
      // indexer, feeder, and belts run
      new JoystickButton(m_controller, OIConstants.kLeftBumper) // left bumper
        .whenPressed(new InstantCommand(m_shooter::runFeed, m_shooter))
        .whenReleased(new InstantCommand(m_shooter::stopFeed, m_shooter))
        .whenPressed(new InstantCommand(m_indexer::runIndexer))
        .whenReleased(new InstantCommand(m_indexer::stopIndexer));
  
        /* // not stable
      // feeder/belts preload
      new JoystickButton(m_controller, OIConstants.kRightBumper) 
      .whenPressed(new ShooterPreload(m_shooter), true)
      .whenReleased(new InstantCommand(m_shooter::stopFeed, m_shooter));
      
  
      // run intake
      new JoystickButton(m_controller, OIConstants.kLeftTrigger) // left trigger button
      .whenPressed(new InstantCommand(m_intake::startIntake))
      .whenReleased(new InstantCommand(m_intake::stopIntake));
  
      // deploy intake
      new JoystickButton(m_controller, OIConstants.kShareButton) // share
      .whenPressed(new InstantCommand(m_intake::deployIntake));
  
      //retract intake
      new JoystickButton(m_controller, OIConstants.kOptionsButton) // options
      .whenPressed(new InstantCommand(m_intake::retractIntake));   
  
      // extend hood piston
      new JoystickButton(m_controller, OIConstants.kTriangleButton) // up POV
      .whenPressed(new InstantCommand(m_shooter::extendHood));
  
      // retract hood piston 
      new JoystickButton(m_controller, OIConstants.kXButton) // down POV
      .whenPressed(m_shooter::retractHood);
  */
  }

}
