// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TestAuto;
import frc.robot.commands.HomeTelescopes;
import frc.robot.commands.LimelightTarget;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.AdvanceFeeder;
import frc.robot.commands.AutoShoot;

import static frc.robot.Constants.OIConstants.*;
import java.time.Instant;
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
  private final Joystick m_fightStick = new Joystick(FIGHT_STICK);
  private final Joystick m_operator = new Joystick(OPERATOR_PORT);

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ClimbSubsystem m_climber = new ClimbSubsystem();
  private final LimelightSubsystem m_limelight = new LimelightSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private SendableChooser<Command> m_autonomousChooser = new SendableChooser();

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

    SmartDashboard.putData("Auto Chooser", m_autonomousChooser);

    PathPlannerTrajectory testTrajectory = PathPlanner.loadPath("Test PP Path", 5.0, 3.0);
    PathPlannerTrajectory straight = PathPlanner.loadPath("STRAIGHT", 1.0, 0.5);

    m_autonomousChooser.setDefaultOption("Test PP Path", 
      new PPSwerveControllerCommand(
       testTrajectory, 
       () -> m_drivetrain.m_odometry.getPoseMeters(), 
       Drivetrain.m_kinematics, 
       new PIDController(0.3, 0, .01), 
       new PIDController(0.3, 0, .01), 
       new ProfiledPIDController(1.5, 0, .01, new Constraints(Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)), 
       (states) -> m_drivetrain.driveFromSpeeds(Drivetrain.m_kinematics.toChassisSpeeds(states)), 
       m_drivetrain));

       m_autonomousChooser.addOption("Multi Step Test", new TestAuto(m_drivetrain, m_intake, m_shooter));

       m_autonomousChooser.addOption("Straight Path", 
       new PPSwerveControllerCommand(
       straight, 
       () -> m_drivetrain.m_odometry.getPoseMeters(), 
       Drivetrain.m_kinematics, 
       new PIDController(0.05, 0, .01), 
       new PIDController(0.03, 0, .01), 
       new ProfiledPIDController(1.5, 0, .01, new Constraints(3.0, 1.5)), 
       (states) -> m_drivetrain.driveFromSpeeds(Drivetrain.m_kinematics.toChassisSpeeds(states)), 
       m_drivetrain));


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

    // operator right bumper lowers intake
    new JoystickButton(m_operator, DS4.R_BUMPER)
    // no requirements, the cylinders can't extend and retract at the same time
    .whenPressed(new InstantCommand(m_intake::extend));

    // operator left bumper raises intake
    new JoystickButton(m_operator, DS4.L_BUMPER)
    // no requirements (see lowering button)
    .whenPressed(new InstantCommand(m_intake::retract));

    // operator left trig. button runs intake fwd.
    new JoystickButton(m_operator, DS4.L_TRIGBUTTON)
    // requires intake subsystem (i think)
    .whileHeld(new InstantCommand(m_intake::intake))
    .whenReleased(new InstantCommand(m_intake::stop));

    // operator right trig. button runs intake rev.
    new JoystickButton(m_operator, DS4.R_TRIGBUTTON)
    // requires intake subsystem (i think)
    .whileHeld(new InstantCommand(m_intake::reverse))
    .whenReleased(new InstantCommand(m_intake::stop));

    // driver circle button runs only shooter
    new JoystickButton(m_driver, DS4.CIRCLE)
    // no requirements
    .whileHeld(new InstantCommand(m_shooter::shootVolts))
    .whenReleased(new InstantCommand(m_shooter::stopFlywheel));

    // operator center pad runs feeder at index speed
    new JoystickButton(m_operator, DS4.CENTER_PAD)
    // no requirements
    .whenPressed(new AdvanceFeeder(m_shooter).withTimeout(0.5), true)
    .whenReleased(new InstantCommand(m_shooter::stopFeeder), false);

    // operator options reverses feeder
    new JoystickButton(m_operator, DS4.OPTIONS)
    .whenPressed(new InstantCommand(m_shooter::feederRev, m_shooter).withTimeout(0.3))
    .whenReleased(new InstantCommand(m_shooter::stopFeeder));

    // operator X button autoshoots low
    new JoystickButton(m_operator, DS4.X)
    .whileHeld(new AutoShoot(() -> m_shooter.shootRPM(3250), m_shooter))
    .whenReleased(new InstantCommand(m_shooter::stopFlywheel).alongWith(new InstantCommand(m_shooter::stopFeeder)));
  
    // driver square button limelight targets
    new JoystickButton(m_driver, DS4.SQUARE)
    .whenPressed(new LimelightTarget(m_limelight, m_drivetrain), true);
    
    // operator square button autoshoots high
    new JoystickButton(m_operator, DS4.SQUARE)
    // requires the shooter
    .whenPressed(new ParallelCommandGroup(
      new SequentialCommandGroup(
        new LimelightTarget(m_limelight, m_drivetrain),
        new InstantCommand(m_shooter::feederFwd)),
      new InstantCommand(() -> m_shooter.shootRPM(3200))
    ))
    .whenReleased(new InstantCommand(m_shooter::stopFlywheel).alongWith(new InstantCommand(m_shooter::stopFeeder)));

    /*
    // Fight Stick X button extends telescope
    new JoystickButton(m_fightStick, FightStick.X)
    .whenPressed(new InstantCommand(m_climber::teleHigh), true)
    .whenReleased(new InstantCommand(m_climber::stopLeft).alongWith(new InstantCommand(m_climber::stopRight)));

    // Fight Stick A button retracts telescope
    new JoystickButton(m_fightStick, FightStick.A)
    .whenPressed(new InstantCommand(m_climber::teleLow), true)
    .whenReleased(new InstantCommand(m_climber::stopLeft).alongWith(new InstantCommand(m_climber::stopRight)));

    // Fight Stick Y button stages tele
    new JoystickButton(m_fightStick, FightStick.Y)
    .whenPressed(new InstantCommand(m_climber::teleStage), true)
    .whenReleased(new InstantCommand(m_climber::stopLeft).alongWith(new InstantCommand(m_climber::stopRight)));

    // Fight stick Right bumper homes tele
    new JoystickButton(m_fightStick, FightStick.R_BUMPER)
    .whenPressed(new HomeTelescopes(m_climber), true)
    .whenReleased(new InstantCommand(m_climber::stopLeft).alongWith(new InstantCommand(m_climber::stopRight)));

    // Fight stick up POV extends pivot
    new POVButton(m_fightStick, 0)
    .whenPressed(new InstantCommand(m_climber::extendPivot));

    // fight stick down POV retracts pivot
    new POVButton(m_fightStick, 180)
    .whenPressed(new InstantCommand(m_climber::retractPivot));
    */
/*
    // Fight stick Left POV extends pivot
      new POVButton(m_fightStick, 0)
      .whenActive(new InstantCommand(m_climber::extendPivot));
    
    // fight stick right POV retracts pivot
      new POVButton(m_fightStick, 180)
      .whenActive(new InstantCommand(m_climber::retractPivot));

    // Fight stick x button retractsleft
    new JoystickButton(m_fightStick, FightStick.X)
    .whileHeld(new InstantCommand(m_climber::setLeftTele))
    .whenReleased(new InstantCommand(m_climber::stopLeft));

    // Fight Stick A button retractsright
    new JoystickButton(m_fightStick, FightStick.A)
    .whileHeld(new InstantCommand(m_climber::retractTele))
    .whenReleased(new InstantCommand(m_climber::stopTele));

    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // returns the command selected in the chooser!!
    return m_autonomousChooser.getSelected();

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
    value = deadband(value, 0.07);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

}
