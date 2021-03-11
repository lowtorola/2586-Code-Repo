/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.FeederPreload;
import frc.robot.commands.FollowPathCommand;
import frc.robot.commands.LimelightTarget;
import frc.robot.commands.ShooterSequence;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final ShooterSubsystem shooterControl = new ShooterSubsystem();
  private final IndexerSubsystem indexerControl = new IndexerSubsystem();
  private final IntakeSubsystem intakeControl = new IntakeSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();

  SendableChooser<Command> m_autonomousChooser = new SendableChooser<>();
  private final String slalomPath = "paths/SlalomPath.wpilib.json";
  private final String barrelRacingPath = "paths/BarrelRacingPath.wpilib.json";
  private final String[] bouncePath = { "paths/BouncePath1.wpilib.json", "paths/BouncePath2.wpilib.json",
      "paths/BouncePath3.wpilib.json", "paths/BouncePath4.wpilib.json" };

  Joystick drive_Stick = new Joystick(OIConstants.kDriveControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * 
   * @throws IOException
   */
  public RobotContainer() throws IOException {
    // Configure the button bindings
    configureButtonBindings();
    robotDrive.setDefaultCommand(
      // A split-stick arcade command, with forward/backward controlled by the left
      // hand, and turning controlled by the right.
      new DefaultDrive(
          robotDrive,
          () -> -drive_Stick.getRawAxis(1),
          () -> drive_Stick.getRawAxis(2)));
    //SmartDashboard.put(shooterControl);

    SmartDashboard.putData(m_autonomousChooser);
    m_autonomousChooser.setDefaultOption("Slalom Path", new FollowPathCommand(robotDrive, slalomPath));
    m_autonomousChooser.addOption("Barrel Racing", new FollowPathCommand(robotDrive, barrelRacingPath));
    m_autonomousChooser.addOption("Bounce Path", new SequentialCommandGroup(
      new FollowPathCommand(robotDrive, bouncePath[0]),
      new FollowPathCommand(robotDrive, bouncePath[1]),
      new FollowPathCommand(robotDrive, bouncePath[2]),
      new FollowPathCommand(robotDrive, bouncePath[3])
    ));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(drive_Stick, OIConstants.kTriangleButton) // just modulate hood
      .whenPressed(shooterControl::extendHood);

    new JoystickButton(drive_Stick, OIConstants.kSquareButton) // just modulate hood
      .whenPressed(shooterControl::retractHood);

    new JoystickButton(drive_Stick, OIConstants.kShooterOnButton) // O
      .whenHeld(new ShooterSequence(shooterControl, indexerControl, intakeControl, limelight), true)
        .whenReleased(new InstantCommand(shooterControl::stopShooter, shooterControl));  

    new JoystickButton(drive_Stick, OIConstants.kFeederOnButton) // left bumper
    .whenPressed(new InstantCommand(shooterControl::runFeeder, shooterControl))
    .whenReleased(new InstantCommand(shooterControl::stopFeeder, shooterControl))
    .whenPressed(new InstantCommand(indexerControl::runIndexer))
    .whenReleased(new InstantCommand(indexerControl::stopIndexer));

    new JoystickButton(drive_Stick, OIConstants.kIntakeOnButton) // left trigger button
    .whenPressed(new InstantCommand(intakeControl::startIntake))
    .whenReleased(new InstantCommand(intakeControl::stopIntake));

    new JoystickButton(drive_Stick, OIConstants.kIntakeDeployButton) // share
    .whenPressed(new InstantCommand(intakeControl::deployIntake));

    new JoystickButton(drive_Stick, OIConstants.kIntakeRetractButton) // options
    .whenPressed(new InstantCommand(intakeControl::retractIntake));
 
    new JoystickButton(drive_Stick, OIConstants.kFeederPreloadButton) // central pad
    .whenPressed(new FeederPreload(shooterControl), true)
    .whenReleased(new InstantCommand(shooterControl::stopFeeder, shooterControl));

    new JoystickButton(drive_Stick, OIConstants.kLimelightAimButton)  // right bumper
    .whenHeld(new LimelightTarget(LimelightConstants.kTargetAngle, robotDrive, limelight));

    new JoystickButton(drive_Stick, OIConstants.kShooterAutoButton) // X
    .whenPressed(new ParallelDeadlineGroup(new ShooterSequence(shooterControl, indexerControl, intakeControl, limelight),
                 new LimelightTarget(LimelightConstants.kTargetAngle, robotDrive, limelight)))
    .whenReleased(new ParallelCommandGroup(
      new InstantCommand(shooterControl::stopShooter, shooterControl),
      new InstantCommand(indexerControl::stopIndexer, indexerControl),
      new InstantCommand(intakeControl::stopIntake, intakeControl),
      new InstantCommand(shooterControl::stopFeeder)
    )); 
  }

  /*Ran via Robot.robotPeriodic */
  public void periodic(){
   shooterControl.printShooterRPM();
   SmartDashboard.putNumber("Measurement", shooterControl.getMeasurement());
   SmartDashboard.putNumber("Left Dist", robotDrive.getLeftDist());
   SmartDashboard.putNumber("Right Dist", robotDrive.getRightDist());
   SmartDashboard.putNumber("Distance to Target", limelight.getDistance());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   // An ExampleCommand will run in autonomous
    return m_autonomousChooser.getSelected();
  }
   
}
