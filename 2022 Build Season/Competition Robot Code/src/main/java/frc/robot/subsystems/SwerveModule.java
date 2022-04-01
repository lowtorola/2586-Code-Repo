// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kDistEncoderResolution = 2048;
  private static final int kTurnEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.MAX_VELOCITY_METERS_PER_SECOND;
  private static final double kModuleMaxAngularAcceleration =
     Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND; // radians per second squared TODO: change back to 2 * Math.PI

  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;
  private boolean driveMotorInverted;

  private double turn_offset;

  // private final CANCoder m_driveEncoder;
  double encUnitMeters = 2 * Math.PI * kWheelRadius / kDistEncoderResolution / 6.75;
  private final CANCoder m_turningEncoder;

  private ProfiledPIDController m_turningPIDController;

  private PIDController m_drivePIDController;

  private SimpleMotorFeedforward m_driveFeedforward;
  private SimpleMotorFeedforward m_turnFeedforward;

  // Swerve mod. state class var
  SwerveModuleState state;

    /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorID CAN output for the drive motor.
   * @param turningMotorID CAN output for the turning motor.
   * @param turningEncoderID CAN input for the turning encoder
   * @param driveMotorInverted whether to invert the drive motor in the module
   * @param drive_kP kP value for drive control
   * @param drive_kS kS value for drive control
   * @param drive_kV kV value for drive control
   * @param drive_kA kA value for drive control
   * @param turn_offset turn offset for module
   * @param turn_kP kP value for turn control
   * @param turn_kD kD value for turn control
   * @param turn_kS kS value for turn feedforward
   * @param turn_kV kV value for turn feedforward
   * @param turn_kA kA value for turn feedforward
   */
  public SwerveModule(
      int driveMotorID,
      int turningMotorID,
      int turningEncoderID,
      boolean driveMotorInverted,
      double drive_kP,
      double drive_kS,
      double drive_kV,
      double drive_kA,
      double turn_offset,
      double turn_kP,
      double turn_kD,
      double turn_kS,
      double turn_kV,
      double turn_kA) {
    m_driveMotor = new WPI_TalonFX(driveMotorID);
    m_turningMotor = new WPI_TalonFX(turningMotorID);
    m_turningEncoder = new CANCoder(turningEncoderID);

    this.turn_offset = turn_offset;
    this.driveMotorInverted = driveMotorInverted;



    m_driveMotor.setInverted(this.driveMotorInverted);
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);

    m_driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 40);
    m_turningMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 40);
    m_turningEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 40);
    m_driveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    m_turningMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 20);

    // Set current limits for drive and turn motors
    m_driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 45, 0.5));
    m_driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5));
    m_turningMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 30, 0.5));
    m_turningMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 35, 0.5));

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    // m_driveEncoder.configFeedbackCoefficient(encUnitMeters, "meters", SensorTimeBase.PerSecond);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    double encUnitRadians = 2 * Math.PI / kTurnEncoderResolution;
    
    m_turningEncoder.configFeedbackCoefficient(encUnitRadians, "radians", SensorTimeBase.PerSecond);
    m_turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_turningEncoder.configMagnetOffset(this.turn_offset);
    m_turningEncoder.configSensorDirection(false);

    m_turningPIDController =
      new ProfiledPIDController(
          turn_kP,
          0,
          turn_kD,
          new TrapezoidProfile.Constraints(
            5.5 * Math.PI, 3.5 * Math.PI));
              // Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    m_turningPIDController.setTolerance(0.005);
    
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    
    // Gains are for example purposes only - must be determined for your own robot!
    m_drivePIDController = new PIDController(drive_kP, 0, 0);

    m_driveFeedforward = new SimpleMotorFeedforward(drive_kS, drive_kV, drive_kA); // kS, kV, kA
    m_turnFeedforward = new SimpleMotorFeedforward(turn_kS, turn_kV, turn_kA); // kS, kV, kA

  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
  //  return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity() * 10 * encUnitMeters, new Rotation2d(getTurnAngle()));
  }

  /**
   * Returns the current angle of the turn encoder in radians
   * @return The current angle of the module in radians
   */
  public double getTurnAngle() {
    return m_turningEncoder.getAbsolutePosition();
  }

  /**
   * Zeroes the turn encoders
   * 
   */
  public void zeroEncoders() {
    m_turningEncoder.setPosition(0);
  }

  /**
   * Get drive motor vel
   * @return drive motor velocity (in raw units)
   */
  public double getDriveSpeed() {
    return m_driveMotor.getSelectedSensorVelocity() * encUnitMeters * 10;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
     state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnAngle()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveMotor.getSelectedSensorVelocity() * 10 * encUnitMeters, state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(getTurnAngle(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(deadband(driveOutput + driveFeedforward, 0.04));
    m_turningMotor.setVoltage(deadband(turnOutput + turnFeedforward, 0.04));
  }

  public ProfiledPIDController getTurnPID() {
    return m_turningPIDController;
  }

  public SwerveModuleState moduleState() {
    return state;
  }

  public double deadband(double x, double db) {
    if(Math.abs(x) < db) {
      return 0;
    } else {
      return x;
    }
  }

}