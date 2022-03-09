// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DefaultDriveCommand;

import static frc.robot.Constants.OIConstants.*;

import java.util.FormatFlagsConversionMismatchException;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Joystick m_driver = new Joystick(DRIVER_PORT);
  // private final Joystick m_fightStick = new Joystick(FIGHT_STICK);
  //private final Joystick m_operator = new Joystick(OPERATOR_PORT);

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ClimbSubsystem m_climber = new ClimbSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrain.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrain,
            () -> -modifyAxis(m_driver.getRawAxis(DS4.L_STICK_Y)) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driver.getRawAxis(DS4.L_STICK_X)) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driver.getRawAxis(DS4.R_STICK_X)) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));


    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Driver share button zeros the gyro
    new JoystickButton(m_driver, DS4.SHARE)
    // no requirements since we don't have to interrupt anything
    .whenPressed(new InstantCommand(m_drivetrain::resetGyro));

    // Driver right bumper lowers intake
    new JoystickButton(m_driver, DS4.R_BUMPER)
    // no requirements, the cylinders can't extend and retract at the same time
    .whenPressed(new InstantCommand(m_intake::extend));

    // Driver left bumper raises intake
    new JoystickButton(m_driver, DS4.L_BUMPER)
    // no requirements (see lowering button)
    .whenPressed(new InstantCommand(m_intake::retract));

    // driver left trig. button runs intake fwd.
    new JoystickButton(m_driver, DS4.L_TRIGBUTTON)
    // requires intake subsystem (i think)
    // FIXME: Make sure requiring the subsystem doesn't break raising/lowering
    .whileHeld(new InstantCommand(m_intake::intake))
    .whenReleased(new InstantCommand(m_intake::stop));

    // driver right trig. button runs intake rev.
    new JoystickButton(m_driver, DS4.R_TRIGBUTTON)
    // requires intake subsystem (i think)
    // FIXME: Make sure requiring the subsystem doesn't break raising/lowering
    .whileHeld(new InstantCommand(m_intake::reverse))
    .whenReleased(new InstantCommand(m_intake::stop));

    // driver circle button runs only shooter
    new JoystickButton(m_driver, DS4.CIRCLE)
    // no requirements
    .whileHeld(new InstantCommand(m_shooter::shootVolts))
    .whenReleased(new InstantCommand(m_shooter::stopFlywheel));

    // driver center pad runs feeder at index speed
    new JoystickButton(m_driver, DS4.CENTER_PAD)
    // no requirements
    .whileHeld(new InstantCommand(m_shooter::feederIndex))
    .whenReleased(new InstantCommand(m_shooter::stopFeeder));

    // driver X button runs shooter and feeder
    new JoystickButton(m_driver, DS4.X)
    // requires the shooter
    .whileHeld(new InstantCommand(m_shooter::shootVolts).alongWith(new InstantCommand(m_shooter::feederFwd)))
    .whenReleased(new InstantCommand(m_shooter::stopFeeder).andThen(new InstantCommand(m_shooter::stopFlywheel)));
/*
    new JoystickButton(m_fightStick, FightStick.X)
    .whileHeld(new InstantCommand(m_climber::extendLeft))
    .whenReleased(new InstantCommand(m_climber::stopLeft));

    new JoystickButton(m_fightStick, FightStick.A)
    .whileHeld(new InstantCommand(m_climber::retractLeft))
    .whenReleased(new InstantCommand(m_climber::stopLeft));

    new JoystickButton(m_fightStick, FightStick.Y)
    .whileHeld(new InstantCommand(m_climber::extendRight))
    .whenReleased(new InstantCommand(m_climber::stopRight));

    new JoystickButton(m_fightStick, FightStick.B)
    .whileHeld(new InstantCommand(m_climber::retractRight))
    .whenReleased(new InstantCommand(m_climber::stopRight));
*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    PathPlannerTrajectory testTrajectory = PathPlanner.loadPath("Test PP Path", Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * 1.0);

    return new PPSwerveControllerCommand(
      testTrajectory, 
      () -> m_drivetrain.m_odometry.getPoseMeters(), 
      Drivetrain.m_kinematics, 
      new PIDController(0.3, 0, .01), 
      new PIDController(0.3, 0, .01), 
      new ProfiledPIDController(0.5, 0, .01, new Constraints(Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 0.8)), 
      (states) -> m_drivetrain.driveFromSpeeds(Drivetrain.m_kinematics.toChassisSpeeds(states)), 
      m_drivetrain);

  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

}
