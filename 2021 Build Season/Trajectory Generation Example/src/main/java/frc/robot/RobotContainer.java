// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.FollowPath;
import frc.robot.commands.FollowPathCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  SendableChooser<Command> m_autonomousChooser = new SendableChooser<>();

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  private final Joystick m_controller = new Joystick(0);

  private final String slalomPath = "paths/SlalomPath.wpilib.json";
  private final String barrelRacingPath = "paths/BarrelRacingPath.wpilib.json";
  private final String[] bouncePath = {
    "paths/BouncePath1.wpilib.json", 
    "paths/BouncePath2.wpilib.json", 
    "paths/BouncePath3.wpilib.json", 
    "paths/BouncePath4.wpilib.json"};
  private final String galacticSearchARed = "paths/GalacticSearch_A_Red.wpilib.json";
  private final String galacticSearchABlue = "paths/GalacticSearch_A_Blue.wpilib.json";
  private final String galacticSearchBRed = "paths/GalacticSearch_B_Red.wpilib.json";
  private final String galacticSearchBBlue = "paths/GalacticSearch_B_Blue.wpilib.json";

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * 
   * @throws IOException
   */
  public RobotContainer() throws IOException {
    // Configure the button bindings
    configureButtonBindings();

    // Configure the button bindings
    configureButtonBindings();
    m_robotDrive.setDefaultCommand(
      // A split-stick arcade command, with forward/backward controlled by the left
      // hand, and turning controlled by the right.
      new DefaultDrive(
          m_robotDrive,
          () -> -m_controller.getRawAxis(1),
          () -> m_controller.getRawAxis(2)));
    //SmartDashboard.put(shooterControl);
  
    SmartDashboard.putData(m_autonomousChooser);

    m_autonomousChooser.setDefaultOption("Slalom Path", new FollowPathCommand(m_robotDrive, slalomPath));

    m_autonomousChooser.addOption("Barrel Racing", new FollowPathCommand(m_robotDrive, barrelRacingPath));

    m_autonomousChooser.addOption("Bounce Path", new SequentialCommandGroup(
      new FollowPathCommand(m_robotDrive, bouncePath[0]),
      new FollowPathCommand(m_robotDrive, bouncePath[1]),
      new FollowPathCommand(m_robotDrive, bouncePath[2]),
      new FollowPathCommand(m_robotDrive, bouncePath[3])));

    m_autonomousChooser.addOption("Galactic Search A Red", 
    new ParallelDeadlineGroup(
      new FollowPathCommand(m_robotDrive, galacticSearchARed), 
      new InstantCommand(m_intake::startIntake)));

    m_autonomousChooser.addOption("Galactic Search A Blue", 
    new ParallelDeadlineGroup(
      new FollowPathCommand(m_robotDrive, galacticSearchABlue), 
      new InstantCommand(m_intake::startIntake)));

    m_autonomousChooser.addOption("Galactic Search B Red", 
      new ParallelDeadlineGroup(
        new FollowPathCommand(m_robotDrive, galacticSearchBRed), 
        new InstantCommand(m_intake::startIntake)));    

    m_autonomousChooser.addOption("Galactic Search B Blue", 
    new ParallelDeadlineGroup(
      new FollowPathCommand(m_robotDrive, galacticSearchBBlue), 
      new InstantCommand(m_intake::startIntake)));

    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_controller, OIConstants.kLeftTrigger)
    .whenPressed(new InstantCommand(m_intake::startIntake))
    .whenReleased(new InstantCommand(m_intake::stopIntake));

    new JoystickButton(m_controller, OIConstants.kOptionsButton)
    .whenPressed(new InstantCommand(m_intake::deployIntake));

    new JoystickButton(m_controller, OIConstants.kShareButton)
    .whenPressed(new InstantCommand(m_intake::retractIntake));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */
  public Command getAutonomousCommand() throws IOException {

  return m_autonomousChooser.getSelected();

   /*
    String trajectoryJSON = "paths/SlalomPath.wpilib.json"; // choose path here
Trajectory chosenPath = new Trajectory();
try {
  Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
  chosenPath = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex) {
  DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
}

    RamseteCommand ramseteCommand = new RamseteCommand(
        chosenPath,
        m_robotDrive::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        m_robotDrive
    );
    */

  }
  /* ran via Robot.periodic() */
  public void periodic(){
   }

}
