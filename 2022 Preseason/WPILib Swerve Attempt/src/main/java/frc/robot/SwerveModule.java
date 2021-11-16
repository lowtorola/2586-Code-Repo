// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/*
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
*/
public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
     2 * Math.PI; // radians per second squared TODO: change back to 2 * Math.PI

  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;
  private boolean driveMotorInverted;

  private double turn_offset;
  private double turn_kP;
  private double turn_kS;
  private double turn_kV;

  // private final CANCoder m_driveEncoder;
  double encUnitMeters = 2 * Math.PI * kWheelRadius / kEncoderResolution;
  private final CANCoder m_turningEncoder;

    /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorID CAN output for the drive motor.
   * @param turningMotorID CAN output for the turning motor.
   * @param turningEncoderID CAN input for the turning encoder
   * @param driveMotorInverted whether to invert the drive motor in the module
   * @param turn_offset turn offset for module
   * @param turn_kP kP value for turn control
   * @param turn_kS kS value for turn feedforward
   * @param turn_kV kV value for turn feedforward
   */
  public SwerveModule(
      int driveMotorID,
      int turningMotorID,
      int turningEncoderID,
      boolean driveMotorInverted,
      double turn_offset,
      double turn_kP,
      double turn_kS,
      double turn_kV) {
    m_driveMotor = new WPI_TalonFX(driveMotorID);
    m_turningMotor = new WPI_TalonFX(turningMotorID);
    m_turningEncoder = new CANCoder(turningEncoderID);

    this.driveMotorInverted = driveMotorInverted;
    this.turn_offset = turn_offset;
    this.turn_kP = turn_kP;
    System.out.println("turn kP: " + turn_kP);
    this.turn_kS = turn_kS;
    this.turn_kV = turn_kV;

    m_driveMotor.setInverted(driveMotorInverted);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    // m_driveEncoder.configFeedbackCoefficient(encUnitMeters, "meters", SensorTimeBase.PerSecond);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    double encUnitRadians = 2 * Math.PI / kEncoderResolution;
    
    m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_turningEncoder.configFeedbackCoefficient(encUnitRadians, "radians", SensorTimeBase.PerSecond);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.672, 0, 0); // from characterization

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          1.18, // TODO: change back to turn_kP
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(.702, 2.69); // kS, kV
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(.62, .24); // kS, kV

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
  //  return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity() * 10 * encUnitMeters, new Rotation2d(m_turningEncoder.getAbsolutePosition() + turn_offset));
  }

  /**
   * Returns the current angle of the turn encoder in radians
   * @return The current angle of the module in radians
   */
  public double getTurnAngle() {
    return m_turningEncoder.getAbsolutePosition() + turn_offset;
  }

  /**
   * Zeroes the turn encoders
   * 
   */
  public void zeroEncoders() {
    m_turningEncoder.setPosition(0);
  }

  public void invertDrive(boolean inverted) {
    m_driveMotor.setInverted(inverted);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getAbsolutePosition() + turn_offset));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveMotor.getSelectedSensorVelocity() * 10 * encUnitMeters, state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition() + turn_offset, state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
