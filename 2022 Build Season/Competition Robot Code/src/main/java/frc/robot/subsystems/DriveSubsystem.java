// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveConstants.*;

public class DriveSubsystem extends SubsystemBase {

  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  // Swerve motors, for PID control
  private final WPI_TalonFX m_frontLeftDrive;
  private final WPI_TalonFX m_frontLeftTurn;
  private final CANCoder m_frontLeftTurnEncoder;
  private final CANCoder m_frontRightTurnEncoder;
  private final CANCoder m_backLeftTurnEncoder;
  private final CANCoder m_backRightTurnEncoder;
  private final WPI_TalonFX m_frontRightDrive;
  private final WPI_TalonFX m_frontRightTurn;
  private final WPI_TalonFX m_backLeftDrive;
  private final WPI_TalonFX m_backLeftTurn;
  private final WPI_TalonFX m_backRightDrive;
  private final WPI_TalonFX m_backRightTurn;


  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation());

  public DriveSubsystem() {

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4SwerveModuleHelper.GearRatio.L2,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );
    // Access drive & turn motors, then set PIDF constants for closed-loop control
    // drive motor
    m_frontLeftDrive = (WPI_TalonFX) m_frontLeftModule.getDriveMotor();
    m_frontLeftDrive.config_kP(0, FRONT_LEFT_MODULE_DRIVE_KP);
    m_frontLeftDrive.config_kI(0, FRONT_LEFT_MODULE_DRIVE_KI);
    m_frontLeftDrive.config_kD(0, FRONT_LEFT_MODULE_DRIVE_KD);
    m_frontLeftDrive.config_kF(0, FRONT_LEFT_MODULE_DRIVE_KF);
    // turn motor
    m_frontLeftTurn = (WPI_TalonFX) m_frontLeftModule.getSteerMotor();
    m_frontLeftTurnEncoder = (CANCoder) m_frontLeftModule.getSteerEncoder();
    m_frontLeftTurn.config_kP(0, FRONT_LEFT_MODULE_TURN_KP);
    m_frontLeftTurn.config_kI(0, FRONT_LEFT_MODULE_TURN_KI);
    m_frontLeftTurn.config_kD(0, FRONT_LEFT_MODULE_TURN_KD);
    m_frontLeftTurn.config_kF(0, FRONT_LEFT_MODULE_TURN_KF);

    // We will do the same for the other modules
    m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );
    // Access drive & turn motors, then set PIDF constants for closed-loop control
    // drive motor
    m_frontRightDrive = (WPI_TalonFX) m_frontRightModule.getDriveMotor();
    m_frontRightDrive.config_kP(0, FRONT_RIGHT_MODULE_DRIVE_KP);
    m_frontRightDrive.config_kI(0, FRONT_RIGHT_MODULE_DRIVE_KI);
    m_frontRightDrive.config_kD(0, FRONT_RIGHT_MODULE_DRIVE_KD);
    m_frontRightDrive.config_kF(0, FRONT_RIGHT_MODULE_DRIVE_KF);
    // turn motor
    m_frontRightTurn = (WPI_TalonFX) m_frontRightModule.getSteerMotor();
    m_frontRightTurnEncoder = (CANCoder) m_frontRightModule.getSteerEncoder();
    m_frontRightTurn.config_kP(0, FRONT_RIGHT_MODULE_TURN_KP);
    m_frontRightTurn.config_kI(0, FRONT_RIGHT_MODULE_TURN_KI);
    m_frontRightTurn.config_kD(0, FRONT_RIGHT_MODULE_TURN_KD);
    m_frontRightTurn.config_kF(0, FRONT_RIGHT_MODULE_TURN_KF);

    m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );
    // Access drive & turn motors, then set PIDF constants for closed-loop control
    // drive motor
    m_backLeftDrive = (WPI_TalonFX) m_backLeftModule.getDriveMotor();
    m_backLeftDrive.config_kP(0, BACK_LEFT_MODULE_DRIVE_KP);
    m_backLeftDrive.config_kI(0, BACK_LEFT_MODULE_DRIVE_KI);
    m_backLeftDrive.config_kD(0, BACK_LEFT_MODULE_DRIVE_KD);
    m_backLeftDrive.config_kF(0, BACK_LEFT_MODULE_DRIVE_KF);
    // turn motor
    m_backLeftTurn = (WPI_TalonFX) m_backLeftModule.getSteerMotor();
    m_backLeftTurnEncoder = (CANCoder) m_frontRightModule.getSteerEncoder();
    m_backLeftTurn.config_kP(0, BACK_LEFT_MODULE_TURN_KP);
    m_backLeftTurn.config_kI(0, BACK_LEFT_MODULE_TURN_KI);
    m_backLeftTurn.config_kD(0, BACK_LEFT_MODULE_TURN_KD);
    m_backLeftTurn.config_kF(0, BACK_LEFT_MODULE_TURN_KF);
    

    m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );
    // Access drive & turn motors, then set PIDF constants for closed-loop control
    // drive motor
    m_backRightDrive = (WPI_TalonFX) m_backRightModule.getDriveMotor();
    m_backRightDrive.config_kP(0, BACK_RIGHT_MODULE_DRIVE_KP);
    m_backRightDrive.config_kI(0, BACK_RIGHT_MODULE_DRIVE_KI);
    m_backRightDrive.config_kD(0, BACK_RIGHT_MODULE_DRIVE_KD);
    m_backRightDrive.config_kF(0, BACK_RIGHT_MODULE_DRIVE_KF);
    // turn motor
    m_backRightTurn = (WPI_TalonFX) m_backRightModule.getSteerMotor();
    m_backRightTurnEncoder = (CANCoder) m_backRightModule.getSteerEncoder();
    m_backRightTurn.config_kP(0, BACK_RIGHT_MODULE_TURN_KP);
    m_backRightTurn.config_kI(0, BACK_RIGHT_MODULE_TURN_KI);
    m_backRightTurn.config_kD(0, BACK_RIGHT_MODULE_TURN_KD);
    m_backRightTurn.config_kF(0, BACK_RIGHT_MODULE_TURN_KF);
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    m_navx.zeroYaw();
    System.out.println("Gyro zeroed");
  }

  public Rotation2d getGyroscopeRotation() {
          
   // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
   return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  public void drivePID(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        m_frontLeftDrive.set(TalonFXControlMode.Velocity, toClosedLoopUnits(states[0].speedMetersPerSecond));
        m_frontRightDrive.set(TalonFXControlMode.Velocity, toClosedLoopUnits(states[1].speedMetersPerSecond));
        m_backLeftDrive.set(TalonFXControlMode.Velocity, toClosedLoopUnits(states[2].speedMetersPerSecond));
        m_backRightDrive.set(TalonFXControlMode.Velocity, toClosedLoopUnits(states[3].speedMetersPerSecond));
        m_frontLeftTurn.set(TalonFXControlMode.Position, states[0].angle.getRadians());
      }

  public double toClosedLoopUnits(double metersPerSecond){
        double RPM = metersPerSecond * 60.0 * (1./0.3191) * 6.75;
        return RPM * 2048.0 / 600.0;
  }

  @Override
  public void periodic() {
          SmartDashboard.putNumber("gyro angle", getGyroscopeRotation().getDegrees());
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  }
}
