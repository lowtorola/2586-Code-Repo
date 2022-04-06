// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OIConstants.DRIVER_PORT;
import static frc.robot.Constants.OIConstants.FIGHT_STICK;
import static frc.robot.Constants.OIConstants.OPERATOR_PORT;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants.DS4;
import frc.robot.Constants.OIConstants.FightStick;
import frc.robot.commands.AdvanceFeeder;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FollowPath;
import frc.robot.commands.HomeTelescopes;
import frc.robot.commands.LimelightTarget;
import frc.robot.commands.RunFeeder;
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

  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();

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

    m_shooter.setDefaultCommand(new AdvanceFeeder(
      m_shooter
    ));

    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData("Autonomous Chooser", m_autoChooser);

    // add auto options to smartdash chooser
    m_autoChooser.addOption("Blue 3 Ball", "Blue 3 ball auto");
    m_autoChooser.addOption("Red 3 Ball", "Red 3 ball auto");
    m_autoChooser.addOption("1 Ball", "Blue 1 ball auto");
    m_autoChooser.addOption("5 Ball", "Blue 5 ball auto");
    m_autoChooser.setDefaultOption("2 Ball", "Blue 2 ball auto");

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
    new JoystickButton(m_driver, DS4.R_BUMPER)
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

    // operator options reverses feeder
    new JoystickButton(m_operator, DS4.OPTIONS)
    .whileHeld(new InstantCommand(m_shooter::feederRev, m_shooter))
    .whenReleased(new InstantCommand(m_shooter::stopFeeder, m_shooter));

    // operator X button autoshoots low
    new JoystickButton(m_operator, DS4.X)
    .whileHeld(new InstantCommand(() -> m_shooter.shootRPM(1600)).alongWith(
      new ConditionalCommand(
        new InstantCommand(m_shooter::feederFwd, m_shooter), 
        new InstantCommand(m_shooter::stopFeeder, m_shooter), 
        m_shooter::atSpeed)))
    .whenReleased(new InstantCommand(m_shooter::stopFlywheel).alongWith(new InstantCommand(m_shooter::stopFeeder)));
  
    // // driver right bumper limelight targets (no shoot)
    // new JoystickButton(m_driver, DS4.R_BUMPER)
    // .whenHeld(new LimelightTarget(m_limelight, m_drivetrain), true);

    // driver square button shoots high
    new JoystickButton(m_driver, DS4.SQUARE)
    .whileHeld(new InstantCommand(() -> m_shooter.shootRPM(2550)).alongWith(
      new ConditionalCommand(
        new InstantCommand(m_shooter::feederFwd), 
        new InstantCommand(m_shooter::stopFeeder), 
        m_shooter::atSpeed)))
    .whenReleased(new InstantCommand(m_shooter::stopFlywheel).alongWith(new InstantCommand(m_shooter::stopFeeder)));

    // driver circle button autoshoots high
    new JoystickButton(m_driver, DS4.CIRCLE)
    .whileHeld(
      new ParallelCommandGroup(
        new InstantCommand(m_limelight::limelightAimConfig),
        new PrintCommand("Spooling"),
        new InstantCommand(() -> m_shooter.shootAuto(m_limelight.getAngleErrorY())),
        new ConditionalCommand(
        new InstantCommand(m_shooter::feederFwd, m_shooter), 
        new InstantCommand(m_shooter::stopFeeder), 
        m_shooter::atSpeed)
    ))
    .whenReleased(
      new ParallelCommandGroup(
        new InstantCommand(m_shooter::stopFlywheel),
        new InstantCommand(m_shooter::stopFeeder),
        new InstantCommand(m_limelight::limelightDriveConfig)
      ));

    // // new auto limelight and shoot code. Only run after doing RPM regression!
    new JoystickButton(m_driver, DS4.L_BUMPER)
    .whenHeld(
      // begin all paths simultaneously
      new ParallelCommandGroup(
        // target, then run the feeder. If needed, just run the feeder unconditionally instead of by atSpeed
        new SequentialCommandGroup(
          new LimelightTarget(m_limelight, m_drivetrain)
          )
    ))
    .whileHeld(new InstantCommand(() -> m_shooter.shootAuto(m_limelight.getAngleErrorY()))
        .alongWith(new ConditionalCommand(
          new RunFeeder(m_shooter), 
          new InstantCommand(m_shooter::stopFeeder), 
          // only feed once we're locked in everywhere !
          ()->(m_shooter.atSpeed() && m_limelight.inPositionX() && m_limelight.inPositionY()) )))
    .whenReleased(new InstantCommand(m_shooter::stopFlywheel).alongWith(new InstantCommand(m_limelight::limelightDriveConfig)));

    // Fight stick Left bumper goes to traverse height
    new JoystickButton(m_fightStick, FightStick.L_BUMPER)
    .whileHeld(new InstantCommand(m_climber::teleTraverse), true);

    // Fight Stick X button extends telescope
    new JoystickButton(m_fightStick, FightStick.X)
    .whileHeld(new InstantCommand(m_climber::teleHigh), true);
  //  .whenReleased(new InstantCommand(m_climber::stopLeft).alongWith(new InstantCommand(m_climber::stopRight)));

    // Fight Stick A button retracts telescope
    new JoystickButton(m_fightStick, FightStick.A)
    .whileHeld(new InstantCommand(m_climber::teleLow), true);
   // .whenReleased(new InstantCommand(m_climber::stopLeft).alongWith(new InstantCommand(m_climber::stopRight)));

    // Fight Stick Y button stages tele
    new JoystickButton(m_fightStick, FightStick.Y)
    .whileHeld(new InstantCommand(m_climber::teleStage), true);
   // .whenReleased(new InstantCommand(m_climber::stopLeft).alongWith(new InstantCommand(m_climber::stopRight)));

   new JoystickButton(m_fightStick, FightStick.B)
   .whileHeld(new InstantCommand(m_climber::stopLeft).alongWith(new InstantCommand(m_climber::stopRight)));

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

    String chosenAuto = m_autoChooser.getSelected();

    SmartDashboard.putString("Chosen Auto", chosenAuto);
  
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(chosenAuto, 5.0, 3.0);

    PathPlannerState initialState = trajectory.getInitialState();
    Pose2d initialPose = new Pose2d(trajectory.getInitialPose().getTranslation(), initialState.holonomicRotation);

    switch(chosenAuto) {
      case "Blue 3 ball auto":
      return new SequentialCommandGroup(
        new InstantCommand(() -> m_shooter.shootRPM(2400)), // TODO: change to limelight shoot!
        new WaitCommand(1),
        new RunFeeder(m_shooter).withTimeout(1),
        new InstantCommand(m_shooter::stopFlywheel),
        new InstantCommand(m_intake::extend),
        new InstantCommand(m_intake::intake),
        new FollowPath(m_drivetrain, trajectory).beforeStarting(new InstantCommand(() -> m_drivetrain.resetOdometry(initialPose))),
        new InstantCommand(m_intake::stop),
        new LimelightTarget(m_limelight, m_drivetrain).withTimeout(1),
        new InstantCommand(() -> m_shooter.shootAuto(m_limelight.getAngleErrorY())),
        new InstantCommand(m_intake::intake),
        new RunFeeder(m_shooter).withTimeout(2.5),
        new InstantCommand(m_shooter::stopFlywheel),
        new InstantCommand(m_intake::stop)
      );
      case "Red 3 ball auto":
      return new SequentialCommandGroup(
        new InstantCommand(() -> m_shooter.shootRPM(2400)), // TODO: change to limelight shoot!
        new WaitCommand(1),
        new RunFeeder(m_shooter).withTimeout(1),
        new InstantCommand(m_shooter::stopFlywheel),
        new InstantCommand(m_intake::extend),
        new InstantCommand(m_intake::intake),
        new FollowPath(m_drivetrain, trajectory).beforeStarting(new InstantCommand(() -> m_drivetrain.resetOdometry(initialPose))),
        new InstantCommand(m_intake::stop),
        new LimelightTarget(m_limelight, m_drivetrain).withTimeout(1),
        new InstantCommand(() -> m_shooter.shootAuto(m_limelight.getAngleErrorY())),
        new InstantCommand(m_intake::intake),
        new RunFeeder(m_shooter).withTimeout(2.5),
        new InstantCommand(m_shooter::stopFlywheel),
        new InstantCommand(m_intake::stop)
      );
      case "Blue 2 ball auto":
      return new SequentialCommandGroup(
        new InstantCommand(() -> m_shooter.shootRPM(2400)), // TODO: change to limelight shoot!
        new WaitCommand(1),
        new RunFeeder(m_shooter).withTimeout(1),
        new InstantCommand(m_shooter::stopFlywheel),
        new InstantCommand(m_intake::extend),
        new InstantCommand(m_intake::intake),
        new FollowPath(m_drivetrain, trajectory).beforeStarting(new InstantCommand(() -> m_drivetrain.resetOdometry(initialPose))),
        new InstantCommand(m_intake::stop),
        // new LimelightTarget(m_limelight, m_drivetrain).withTimeout(1), // Shouldn't be necessary for just a 2 ball
        new InstantCommand(() -> m_shooter.shootAuto(m_limelight.getAngleErrorY())),
        new InstantCommand(m_intake::intake),
        new RunFeeder(m_shooter).withTimeout(2),
        new InstantCommand(m_shooter::stopFlywheel),
        new InstantCommand(m_intake::stop)
      );
      case "Blue 1 ball auto":
      return new SequentialCommandGroup(
        new InstantCommand(()->m_shooter.shootRPM(2400)), // TODO: change to limelight shoot?
        new WaitCommand(1),
        new RunFeeder(m_shooter).withTimeout(1),
        new InstantCommand(m_shooter::stopFlywheel),
        new FollowPath(m_drivetrain, trajectory).beforeStarting(new InstantCommand(() -> m_drivetrain.resetOdometry(initialPose)))
      );
      case "Blue 5 ball auto":

      PathPlannerTrajectory step1 = PathPlanner.loadPath("Blue 5 ball step 1", 5.0, 3.0);
      PathPlannerTrajectory step2 = PathPlanner.loadPath("Blue 5 ball step 2", 6.0, 3.0);
      PathPlannerTrajectory step3 = PathPlanner.loadPath("Blue 5 ball step 3", 6.0, 3.0);

      PathPlannerState first_state = step1.getInitialState();
      Pose2d first_pose = new Pose2d(step1.getInitialPose().getTranslation(), first_state.holonomicRotation);

      return new SequentialCommandGroup(
        // Step 1: first 3 balls
        new InstantCommand(m_limelight::limelightAimConfig),
        new InstantCommand(() -> m_shooter.shootAuto(m_limelight.getAngleErrorY())),
        new WaitCommand(0.5),
        new RunFeeder(m_shooter).withTimeout(1.5),
        new InstantCommand(m_shooter::stopFlywheel),
        new InstantCommand(m_intake::extend),
        new InstantCommand(m_intake::intake),
        new FollowPath(m_drivetrain, step1).withTimeout(6.5).beforeStarting(new InstantCommand(() -> m_drivetrain.resetOdometry(first_pose))), // path following to pick up 2 balls
        new InstantCommand(m_intake::stop),
      //  new LimelightTarget(m_limelight, m_drivetrain).withTimeout(1), // try no LL target to keep path accurate
        new InstantCommand(() -> m_shooter.shootAuto(m_limelight.getAngleErrorY())),
        new WaitCommand(1),
        new InstantCommand(m_intake::intake),
        new RunFeeder(m_shooter).withTimeout(1.5),
        new InstantCommand(m_shooter::stopFlywheel),
        // Step 2: drive back to terminal
        
        new FollowPath(
          m_drivetrain, 
          step2).withTimeout(4), // path following back to terminal
        new WaitCommand(0.5), // give human player a chance to roll er in
       
        // Step 3: drive to hub to shoot
        new FollowPath(m_drivetrain, step3).withTimeout(4),
        new InstantCommand(m_intake::stop),
        new LimelightTarget(m_limelight, m_drivetrain).withTimeout(1.5), // commented out for testing we're GONNA need this
        new InstantCommand(() -> m_shooter.shootAuto(m_limelight.getAngleErrorY())),
        new WaitCommand(0.5),
        new InstantCommand(m_intake::intake),
        new RunFeeder(m_shooter).withTimeout(1.5),
        new InstantCommand(m_shooter::stopFlywheel),
        new InstantCommand(m_intake::stop)
      );
      default:
      return null;

    }






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
    value = deadband(value, 0.06);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

}
