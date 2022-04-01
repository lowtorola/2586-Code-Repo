// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OIConstants.DRIVER_PORT;
import static frc.robot.Constants.OIConstants.FIGHT_STICK;
import static frc.robot.Constants.OIConstants.OPERATOR_PORT;

import java.time.Instant;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants.DS4;
import frc.robot.Constants.OIConstants.FightStick;
import frc.robot.commands.AdvanceFeeder;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.HomeTelescopes;
import frc.robot.commands.LimelightTarget;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


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
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ClimbSubsystem m_climber = new ClimbSubsystem();
  private final LimelightSubsystem m_limelight = new LimelightSubsystem();

//  private SendableChooser<String> m_autonomousChooser = new SendableChooser();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrain.setDefaultCommand(new DriveCommand(
            m_drivetrain,
            () -> -modifyAxis(m_driver.getRawAxis(DS4.L_STICK_Y)) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driver.getRawAxis(DS4.L_STICK_X)) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driver.getRawAxis(DS4.R_STICK_X)) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            true
    ));

    // SmartDashboard.putData("Auto Chooser", m_autonomousChooser);

    // m_autonomousChooser.setDefaultOption("Blue 1 Ball", "Blue 1 ball auto");
    // m_autonomousChooser.addOption("Blue 2 ball", "Blue 2 ball auto");
    // m_autonomousChooser.addOption("Red 1 ball", "Red 1 ball auto");
    // m_autonomousChooser.addOption("Red 2 ball", "Red 2 ball auto");

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

    // Driver left bumper turns off field-oriented drive
    new JoystickButton(m_driver, DS4.L_BUMPER)
    .whenHeld(new DriveCommand(
      m_drivetrain,
      () -> -modifyAxis(m_driver.getRawAxis(DS4.L_STICK_Y)) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(m_driver.getRawAxis(DS4.L_STICK_X)) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(m_driver.getRawAxis(DS4.R_STICK_X)) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      false
  ));

    // operator right bumper lowers intake when pressed, raises when released
    new JoystickButton(m_operator, DS4.R_BUMPER)
    // no requirements, the cylinders can't extend and retract at the same time
    .whenPressed(new InstantCommand(m_intake::extend))
    .whenReleased(new InstantCommand(m_intake::retract));

    // operator left bumper toggles intake
    new JoystickButton(m_operator, DS4.L_BUMPER)
    .whenPressed(new InstantCommand(m_intake::toggleIntake));

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

    // operator center pad advances feeder
    new JoystickButton(m_operator, DS4.CENTER_PAD)
    // no requirements
    .whenPressed(new AdvanceFeeder(m_shooter), true)
    .whenReleased(new InstantCommand(m_shooter::stopFeeder), false);

    // operator options reverses feeder
    new JoystickButton(m_operator, DS4.OPTIONS)
    .whenPressed(new InstantCommand(m_shooter::feederRev, m_shooter))
    .whenReleased(new InstantCommand(m_shooter::stopFeeder));

    // operator X button autoshoots low
    new JoystickButton(m_operator, DS4.X)
    .whileHeld(new AutoShoot(() -> m_shooter.shootRPM(1400), m_shooter))
    .whenReleased(new InstantCommand(m_shooter::stopFlywheel).alongWith(new InstantCommand(m_shooter::stopFeeder)));
  
    // driver square button limelight targets (no shoot)
    new JoystickButton(m_driver, DS4.SQUARE)
    .whenHeld(new LimelightTarget(m_limelight, m_drivetrain), true)
    .whenReleased(new InstantCommand(m_limelight::limelightDriveConfig));
    
    // operator X button autoshoots high
    new JoystickButton(m_operator, DS4.SQUARE)
    .whileHeld(new AutoShoot(() -> m_shooter.shootRPM(2500), m_shooter))
    .whenReleased(new InstantCommand(m_shooter::stopFlywheel).alongWith(new InstantCommand(m_shooter::stopFeeder)));

    // operator square button autoshoots high (target while spooling then run feeder unconditionally)

    // new JoystickButton(m_operator, DS4.SQUARE)
    // // requires the shooter
    // .whenHeld(new ParallelCommandGroup(
    //   new SequentialCommandGroup(
    //     new LimelightTarget(m_limelight, m_drivetrain),
    //     new InstantCommand(m_shooter::feederFwd)),
    //   new InstantCommand(() -> m_shooter.shootRPM(2550))
    // ))
    // .whenReleased(new InstantCommand(m_shooter::stopFlywheel).alongWith(new InstantCommand(m_shooter::stopFeeder)).alongWith(new InstantCommand(m_limelight::limelightDriveConfig)));
    
    // Fight Stick X button extends telescope
    new JoystickButton(m_fightStick, FightStick.X)
    .whenPressed(new InstantCommand(m_climber::teleHigh), true);
  //  .whenReleased(new InstantCommand(m_climber::stopLeft).alongWith(new InstantCommand(m_climber::stopRight)));

    // Fight Stick A button retracts telescope
    new JoystickButton(m_fightStick, FightStick.A)
    .whenPressed(new InstantCommand(m_climber::teleLow), true);
   // .whenReleased(new InstantCommand(m_climber::stopLeft).alongWith(new InstantCommand(m_climber::stopRight)));

    // Fight Stick Y button stages tele
    new JoystickButton(m_fightStick, FightStick.Y)
    .whenPressed(new InstantCommand(m_climber::teleStage), true);
   // .whenReleased(new InstantCommand(m_climber::stopLeft).alongWith(new InstantCommand(m_climber::stopRight)));

   new JoystickButton(m_fightStick, FightStick.B)
   .whenPressed(new InstantCommand(m_climber::stopLeft).alongWith(new InstantCommand(m_climber::stopRight)));

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
    
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("Blue 3 ball auto", 4.0, 2.0);

    PathPlannerState initialState = trajectory.getInitialState();
    Pose2d startingPose = new Pose2d(trajectory.getInitialPose().getTranslation(), initialState.holonomicRotation);

    PPSwerveControllerCommand blue3Ball = 
    new PPSwerveControllerCommand(
        trajectory, 
        () -> m_drivetrain.getPose(), 
        Drivetrain.m_kinematics, 
        new PIDController(0.5, 0, .01), // tune this
        new PIDController(0.5, 0, .01), // and this
        new ProfiledPIDController(0.5, 0, .01, new Constraints(5, 2.5)), // and this....
        (states) -> m_drivetrain.driveFromSpeeds(Drivetrain.m_kinematics.toChassisSpeeds(states), false), 
        m_drivetrain);

    return blue3Ball.beforeStarting(new InstantCommand(() -> m_drivetrain.resetOdometry(startingPose)));

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
    value = deadband(value, 0.04);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

}
