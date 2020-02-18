/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.ShooterSmPID;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  public CANSparkMax leftMaster, leftSlave, rightMaster, rightSlave, shooter;
  CANEncoder shooterEncoder;
  Spark shooterFeeder, indexerLeft, indexerRight, intake;

  ShooterSmPID shooterPID;

  Joystick driveStick, operatorStick;

  DifferentialDrive drive;

  DoubleSolenoid intakeHeight;

  DigitalInput bbShooterFeeder;
  // DigitalInput bbShooterExit;

  boolean shooterFeeder_state, shooterExit_state;

  double shooterRPM;

  int indexStep;

  @Override
  public void robotInit() {

    leftMaster = new CANSparkMax(1, MotorType.kBrushless);
    leftSlave = new CANSparkMax(2, MotorType.kBrushless);
    rightMaster = new CANSparkMax(3, MotorType.kBrushless);
    rightSlave = new CANSparkMax(4, MotorType.kBrushless);
    shooter = new CANSparkMax(9, MotorType.kBrushless);

    shooterEncoder = shooter.getEncoder();

    indexerLeft = new Spark(0);
    indexerRight = new Spark(1);
    intake = new Spark(2);
    shooterFeeder = new Spark(3);

    driveStick = new Joystick(0);
    operatorStick = new Joystick(1);

    intakeHeight = new DoubleSolenoid(0, 1);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    leftMaster.setInverted(false);
    rightMaster.setInverted(false);

    shooterPID.setP(ShooterSmPID.KP);
    shooterPID.setFF(ShooterSmPID.KF);
    shooterPID.setSmartMotionMaxVelocity(ShooterSmPID.MAXVEL, 0);

    drive = new DifferentialDrive(leftMaster, rightMaster);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    shooterLogic();
    feederLogic();
    indexerLogic();

  drive.arcadeDrive(-driveStick.getRawAxis(1), driveStick.getRawAxis(2));

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  public void robotPeriodic() {
    SmartDashboard.putNumber("Shooter Current", shooter.getOutputCurrent());
    SmartDashboard.putNumber("Shooter RPM", shooterEncoder.getVelocity());
    shooterRPM = shooterEncoder.getVelocity();
    shooterFeeder_state = bbShooterFeeder.get();
    // shooterExit_state = bbShooterExit.get();
  }

  public void shooterLogic() {
    if (operatorStick.getRawButton(14)) {
      shooterSet();
    }
  }

  public void indexerLogic() {
    if (operatorStick.getRawButton(14)) {
      indexerSet();
    }
  }

  public void feederLogic() {
    if (operatorStick.getRawButton(2) && shooterFeeder_state) {
      creepFeederSet();
    } else if (operatorStick.getRawButton(14) && shooterRPM > 2200) {
      shootFeederSet();
    } else {
      feederStop();
    }
  }

  public void intakeSpeedControl() {
    if(driveStick.getRawButton(7)) {
      intakeIntake();
    } else if (driveStick.getRawButton(8)) {
      intakeOuttake();
    } else {
      intakeStop();
    }
  }

  public void intakeHeightControl() {
    if (driveStick.getRawButton(9)) {

    }
  }

  public void shootFeederSet() {
    shooterFeeder.set(.6);
  }

  public void creepFeederSet() {
    shooterFeeder.set(.2);
  }

  public void feederStop() {
    shooterFeeder.set(0);
  }

  public void indexerSet() {
      indexerLeft.set(0.4);
      indexerRight.set(0.4);
  }

  public void shooterSet() {
    shooterPID.setReference(shooterRPM, ControlType.kSmartVelocity, 0);
  }

  public void intakeIntake() {
    intake.set(0.5);
  }

  public void intakeOuttake() {
    intake.set(-0.5);
  }

  public void intakeStop() {
    intake.set(0);
  }

}
