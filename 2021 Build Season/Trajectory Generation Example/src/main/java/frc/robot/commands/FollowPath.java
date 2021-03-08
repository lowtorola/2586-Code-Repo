// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

/** An example command that uses an example subsystem. */
public class FollowPath extends RamseteCommand {

    String useFilePath;

    public FollowPath(DriveSubsystem driveSubsystem, String filePath) throws IOException {

    super(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(filePath)),
    driveSubsystem::getPose,
    new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts,
                               DriveConstants.kvVoltSecondsPerMeter,
                               DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics,
    driveSubsystem::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    driveSubsystem::tankDriveVolts,
    driveSubsystem);
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    useFilePath = filePath;
  }

  public Trajectory getChosenPath() throws IOException {
    return TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(useFilePath));
  }


}
