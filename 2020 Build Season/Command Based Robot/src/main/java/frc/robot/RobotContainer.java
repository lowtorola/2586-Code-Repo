/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.FeederPreload;
import frc.robot.commands.LimelightAim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem shooterControl = new ShooterSubsystem();
  private final IndexerSubsystem indexerControl = new IndexerSubsystem();
 // private final PneumaticSubsystem compControl = new PneumaticSubsystem();
//  private final IntakeSubsystem intakeControl = new IntakeSubsystem();

  Joystick drive_Stick = new Joystick(OIConstants.kDriveControllerPort);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_robotDrive.setDeadband(DriveConstants.kDriveDeadband);
    m_robotDrive.setDefaultCommand(
      // A split-stick arcade command, with forward/backward controlled by the left
      // hand, and turning controlled by the right.
      new DefaultDrive(
          m_robotDrive,
          () -> -drive_Stick.getRawAxis(1),
          () -> drive_Stick.getRawAxis(2)));

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(drive_Stick, OIConstants.kShooterOnButton)
      .whenPressed(new InstantCommand(shooterControl::enable, shooterControl))
      .whenReleased(new InstantCommand(shooterControl::disable, shooterControl));      

    new JoystickButton(drive_Stick, OIConstants.kFeederOnButton)
    .whenPressed(new InstantCommand(shooterControl::runFeeder, shooterControl))
    .whenReleased(new InstantCommand(shooterControl::stopFeeder, shooterControl))
    .whenPressed(new InstantCommand(indexerControl::runIndexer))
    .whenReleased(new InstantCommand(indexerControl::stopIndexer));
/**
    new JoystickButton(drive_Stick, OIConstants.kIntakeOnButton)
    .whenPressed(new InstantCommand(intakeControl::startIntake))
    .whenReleased(new InstantCommand(intakeControl::stopIntake));

    new JoystickButton(drive_Stick, OIConstants.kIntakeDeployButton)
    .whenPressed(new InstantCommand(intakeControl::deployIntake));

    new JoystickButton(drive_Stick, OIConstants.kIntakeRetractButton)
    .whenPressed(new InstantCommand(intakeControl::retractIntake));
*/
    new JoystickButton(drive_Stick, OIConstants.kFeederPreloadButton)
    .whenHeld(new FeederPreload(shooterControl), true);

    new JoystickButton(drive_Stick, OIConstants.kLimelightAimButton) 
    .whenHeld(new LimelightAim(LimelightConstants.kTargetAngle, m_robotDrive), true);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 /** public Command getAutonomousCommand() {
  *  // An ExampleCommand will run in autonomous
   * return m_autoCommand;
  }
   */
}
